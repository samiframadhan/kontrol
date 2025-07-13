# linefollowing_rs.py (Refactored as a ManagedNode)
import cv2
import math
import numpy as np
import zmq
import time
import logging
import yaml
import threading
from pyrealsense2 import pyrealsense2 as rs
from dataclasses import dataclass
from abc import ABC, abstractmethod
import steering_command_pb2

# Assuming managed_node.py is in the same directory or accessible in the Python path
from managednode import ManagedNode
from camera import RealSenseCamera, OpenCVCamera, Camera, ZMQCamera

# -------------------------------------------------------------------
# --- 1. CONFIGURATION & LOGGING (Mostly unchanged)
# -------------------------------------------------------------------

def load_config(config_path='linedetection.yaml'):
    """Loads configuration from a YAML file."""
    logging.info(f"Loading configuration from {config_path}...")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    ld_config = config['lane_detection']
    ld_config['hsv_lower_np'] = np.array(ld_config['hsv_lower'])
    ld_config['hsv_upper_np'] = np.array(ld_config['hsv_upper'])
    ld_config['hsv_lower_gpu_tuple'] = tuple(ld_config['hsv_lower'])
    ld_config['hsv_upper_gpu_tuple'] = tuple(ld_config['hsv_upper'])
    logging.info("Configuration loaded successfully.")
    return config

def setup_logging():
    """Configures the logging for the application."""
    # The ManagedNode class already sets up basic logging,
    # but we can call this if specific formatting is desired.
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# -------------------------------------------------------------------
# --- 2. LANE DETECTION CLASSES
# -------------------------------------------------------------------

class StanleyController:
    """Calculates steering angle using the Stanley control method."""
    @staticmethod
    def calculate_steering(cross_track_error, heading_error, speed, k, epsilon):
        cross_track_term = math.atan2(k * cross_track_error, speed + epsilon)
        return heading_error + cross_track_term

class LaneDetector:
    """Handles the full CV pipeline to detect lane and calculate errors using GPU."""
    def __init__(self, config: dict, use_cuda: bool):
        self.config = config
        self.ld_config = config['lane_detection'] # Shortcut to lane detection params
        self.use_cuda = use_cuda
        if self.use_cuda:
            self.gpu_frame = cv2.cuda_GpuMat()
            self.gpu_warped = cv2.cuda_GpuMat()
            self.gpu_hsv = cv2.cuda_GpuMat()
            self.gpu_mask = cv2.cuda_GpuMat()
            logging.info("LaneDetector initialized with CUDA support.")
        else:
            logging.info("LaneDetector initialized for CPU processing.")

    def _get_perspective_transform(self, img_shape):
        H, W = img_shape
        warp = self.ld_config['perspective_warp']
        mirror_point = 1 - warp
        src = np.float32([[W * warp, H], [0, 0], [W, 0], [W * mirror_point, H]])
        dst = np.float32([[0, H], [0, 0], [W, 0], [W, H]])
        return cv2.getPerspectiveTransform(dst, src)

    def _find_lane_contours(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        centers = []
        min_area = self.ld_config['min_contour_area']
        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > min_area:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append({'x': cx, 'y': cy, 'area': M['m00']})
        return contours, centers

    @staticmethod
    def _region_of_interest(img, vertices):
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, vertices, 255)
        return cv2.bitwise_and(img, mask)

    def _calculate_perpendicular_error(self, m, b, H, W):
        """
        Calculates the perpendicular distance from the virtual axle center to the detected lane line.
        This version accounts for the physical offset between the camera's view and the axle.
        """
        # --- MODIFICATION START ---
        
        # Get conversion and offset from config. Use .get() for safety if param is missing.
        pixels_per_meter = 1.0 / self.ld_config.get('pixels_to_meters', 0.0035) # Meters to pixels
        axle_offset_m = self.ld_config.get('axle_to_bottom_frame_m', 0.0)
        
        # Calculate the vertical offset in pixels
        y_offset_pixels = int(axle_offset_m * pixels_per_meter)
        
        # Define the virtual axle point on the bird's-eye view image
        # This is our new reference point for all error calculations.
        virtual_axle_point = (W // 2, H - y_offset_pixels)
        
        # Calculate the point on the detected lane line that is perpendicular to our virtual axle point
        if abs(m) < 1e-6: # Avoid division by zero for horizontal lines
            x_int, y_int = virtual_axle_point[0], b
        else:
            perp_m = -1 / m
            perp_b = virtual_axle_point[1] - perp_m * virtual_axle_point[0]
            x_int = (perp_b - b) / (m - perp_m)
            y_int = m * x_int + b
            
        intersection_point = (int(x_int), int(y_int))
        
        # Calculate the distance (error) in pixels
        error_pixels = math.hypot(x_int - virtual_axle_point[0], y_int - virtual_axle_point[1])
        
        # Determine the sign of the error. Instead of checking the line's x-intercept at the
        # image bottom (y=H), we check it at the virtual axle's y-coordinate.
        virtual_axle_y_coord = H - y_offset_pixels
        x_at_axle_line = (virtual_axle_y_coord - b) / m if abs(m) > 1e-6 else float('inf')
        
        if x_at_axle_line < W // 2:
            error_pixels = -error_pixels
            
        # --- MODIFICATION END ---
            
        return error_pixels, intersection_point

    def process_frame(self, img):
        H, W = img.shape[:2]
        M = self._get_perspective_transform((H, W))

        if self.use_cuda:
            self.gpu_frame.upload(img)
            self.gpu_warped = cv2.cuda.warpPerspective(self.gpu_frame, M, (W, H), flags=cv2.INTER_LINEAR)
            self.gpu_hsv = cv2.cuda.cvtColor(self.gpu_warped, cv2.COLOR_BGR2HSV)
            self.gpu_mask = cv2.cuda.inRange(self.gpu_hsv, self.ld_config['hsv_lower_gpu_tuple'], self.ld_config['hsv_upper_gpu_tuple'])
            mask_yellow = self.gpu_mask.download()
            img_warped = self.gpu_warped.download()
        else:
            img_warped = cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_LINEAR)
            hsv_img = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(hsv_img, self.ld_config['hsv_lower_np'], self.ld_config['hsv_upper_np'])

        all_contours, all_centers = [], []
        num_segments = self.ld_config['segments']
        region_height = H // num_segments
        for i in range(num_segments):
            segment_vertices = np.array([[(0, H - (i + 1) * region_height), (W, H - (i + 1) * region_height),
                                          (W, H - i * region_height), (0, H - i * region_height)]], dtype=np.int32)
            segment_mask = self._region_of_interest(mask_yellow, segment_vertices)
            contours, centers = self._find_lane_contours(segment_mask)
            all_contours.extend(contours)
            all_centers.extend(centers)

        centers_sorted = sorted(all_centers, key=lambda c: -c['y'])
        pipeline_data = {'warped_image': img_warped, 'color_mask': mask_yellow, 'contours': all_contours,
                         'centers': all_centers, 'cross_track_error': 0.0, 'heading_error': 0.0,
                         'lane_points': [], 'cte_intersection_point': None}

        if len(centers_sorted) >= 2:
            p1 = (centers_sorted[0]['x'], centers_sorted[0]['y'])
            p2 = (centers_sorted[1]['x'], centers_sorted[1]['y'])
            pipeline_data['lane_points'] = [p1, p2]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]

            if abs(dx) > 1e-6:
                m = dy / dx
                b = p1[1] - m * p1[0]
                pipeline_data['heading_error'] = (math.pi / 2) - math.atan2(-dy, dx)
                error_pixels, intersection_point = self._calculate_perpendicular_error(m, b, H, W)
                pipeline_data['cross_track_error'] = error_pixels * self.ld_config['pixels_to_meters']
                pipeline_data['cte_intersection_point'] = intersection_point
        else:
            logging.debug("Not enough lane points detected to calculate errors.")

        return pipeline_data

class Visualizer:
    """Handles all OpenCV drawing operations."""
    def draw_pipeline_data(self, image, data, steering_angle):
        H, W = image.shape[:2]
        cv2.drawContours(image, data['contours'], -1, (255, 0, 255), 1)
        for center in data['centers']: cv2.circle(image, (center['x'], center['y']), 5, (0, 255, 0), -1)
        if data['lane_points']: cv2.line(image, data['lane_points'][0], data['lane_points'][1], (255, 0, 0), 3)
        if data['cte_intersection_point']:
            bottom_center = (W // 2, H); cv2.line(image, bottom_center, data['cte_intersection_point'], (255, 0, 255), 2)
        arrow_length = 100; x0, y0 = W // 2, H
        arrow_angle = (math.pi / 2) - steering_angle
        x1, y1 = int(x0 + arrow_length * math.cos(arrow_angle)), int(y0 - arrow_length * math.sin(arrow_angle))
        cv2.arrowedLine(image, (x0, y0), (x1, y1), (0, 0, 255), 3, tipLength=0.2)
        cv2.putText(image, f"CTE: {data['cross_track_error']:.3f} m", (10, H - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"HE: {math.degrees(data['heading_error']):.1f} deg", (10, H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"Steer: {math.degrees(steering_angle):.1f} deg", (10, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        return image

# -------------------------------------------------------------------
# --- 3. MANAGED NODE IMPLEMENTATION
# -------------------------------------------------------------------

class LineFollowingNode(ManagedNode):
    """
    A managed node for running the line following vision pipeline.
    """
    def __init__(self, node_name: str, config_path: str = 'linedetection.yaml'):
        super().__init__(node_name)
        self.config_path = config_path
        self.config = None
        self.camera = None
        self.lane_detector = None
        self.visualizer = None
        self.pub_socket = None
        self.result_writer = None
        
        # Threading control for the processing loop
        self.processing_thread = None
        self.active_event = threading.Event()

    def on_configure(self) -> bool:
        """Load configuration and initialize all components."""
        self.logger.info("Configuring Line Following Node...")
        try:
            self.config = load_config(self.config_path)
            cam_config = self.config['camera']
            zmq_config = self.config['zmq']

            # Use the ZMQ context from the parent ManagedNode class
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(zmq_config['pub_steer_url'])
            
            # Note: The ZMQ Camera needs a context. It will use the one from ManagedNode.
            source_type = cam_config.get('source_type', '').lower()
            if source_type == 'realsense':
                self.camera = RealSenseCamera(cam_config)
            elif source_type == 'opencv':
                self.camera = OpenCVCamera(cam_config, source=cam_config.get('opencv_source'))
            elif source_type == 'zmq':
                self.camera = ZMQCamera(cam_config, zmq_config, self.context)
            else:
                self.logger.error(f"Invalid camera source_type: {source_type}")
                return False

            use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
            self.logger.info(f"CUDA Available: {use_cuda}. {'Using GPU.' if use_cuda else 'Running on CPU.'}")

            self.lane_detector = LaneDetector(self.config, use_cuda)
            self.visualizer = Visualizer()
            
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.result_writer = cv2.VideoWriter(
                cam_config['video_output_file'], fourcc, cam_config['frame_fps'],
                (cam_config['frame_width'] * 2, cam_config['frame_height'])
            )
            self.logger.info("Configuration successful.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during configuration: {e}")
            return False

    def on_activate(self) -> bool:
        """Start the camera and the processing loop."""
        self.logger.info("Activating Node...")
        try:
            self.camera.start()
            self.active_event.set() # Signal the processing loop to run
            self.processing_thread = threading.Thread(target=self._processing_loop, name=f"{self.node_name}_ProcessingThread")
            self.processing_thread.start()
            self.logger.info("Node activated.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during activation: {e}")
            return False

    def on_deactivate(self) -> bool:
        """Stop the processing loop and the camera."""
        self.logger.info("Deactivating Node...")
        try:
            self.active_event.clear() # Signal the processing loop to stop
            if self.processing_thread:
                self.processing_thread.join(timeout=2.0)
            if self.camera:
                self.camera.stop()
            self.logger.info("Node deactivated.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during deactivation: {e}")
            return False

    def on_shutdown(self) -> bool:
        """Release all resources."""
        self.logger.info("Shutting down Node...")
        # Ensure deactivation sequence is called first
        if self.state == "active":
            self.on_deactivate()
            
        try:
            if self.result_writer:
                self.result_writer.release()
            if self.pub_socket:
                self.pub_socket.close()
            cv2.destroyAllWindows()
            # The parent run() method handles context termination
            self.logger.info("Shutdown successful.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during shutdown: {e}")
            return False
            
    def _processing_loop(self):
        """The main loop for frame processing and command publishing."""
        start_time = time.time()
        frame_count = 0
        cam_config = self.config['camera']
        zmq_config = self.config['zmq']
        sc_config = self.config['stanley_controller']

        while self.active_event.is_set() and not self.shutdown_event.is_set():
            frame = self.camera.get_frame()
            if frame is None:
                # In some cases (e.g., ZMQ), this is normal. Avoid busy-waiting.
                time.sleep(0.01)
                continue
            
            # --- Core processing logic ---
            lane_data = self.lane_detector.process_frame(frame)

            steering_angle_rad = StanleyController.calculate_steering(
                lane_data['cross_track_error'],
                lane_data['heading_error'],
                sc_config['assumed_speed_for_calc'],
                sc_config['gain'],
                sc_config['speed_epsilon']
            )
            steering_angle_deg = math.degrees(steering_angle_rad)

            # --- Publish steering command ---
            command = steering_command_pb2.SteeringCommand()
            command.auto_steer_angle = steering_angle_deg
            serialized_command = command.SerializeToString()
            self.pub_socket.send_string(zmq_config['steer_topic'], flags=zmq.SNDMORE)
            self.pub_socket.send(serialized_command)

            # --- Visualization ---
            warped_view = lane_data.get('warped_image', np.zeros_like(frame))
            annotated_warped = self.visualizer.draw_pipeline_data(warped_view, lane_data, steering_angle_rad)
            stacked_output = np.hstack((frame, annotated_warped))
            
            if cam_config.get('use_display', False):
                cv2.imshow('Lane Following View', stacked_output)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.logger.info("'q' pressed in display window, initiating shutdown.")
                    self.shutdown_event.set() # Signal main run loop to exit
                    break
            
            self.result_writer.write(stacked_output)
            
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed_time = time.time() - start_time
                fps = frame_count / elapsed_time
                self.logger.info(f"Processing FPS: {fps:.2f}")
        
        self.logger.info("Processing loop terminated.")


# -------------------------------------------------------------------
# --- 4. MAIN EXECUTION
# -------------------------------------------------------------------

def main():
    """
    Main entry point: Creates and runs the LineFollowingNode.
    """
    setup_logging()
    # The node name should be unique in the orchestrated system
    line_follower = LineFollowingNode(node_name="line_follower_node")
    line_follower.run()

if __name__ == "__main__":
    main()