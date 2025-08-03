# linefollowing_node.py (Updated with speed feedback and velocity output)
import cv2
import math
import numpy as np
import zmq
import time
import logging
import threading
from pyrealsense2 import pyrealsense2 as rs
from dataclasses import dataclass
from abc import ABC, abstractmethod
import steering_command_pb2

from managednode import ManagedNode
from config_mixin import ConfigMixin
from camera_node import ZMQCamera

def setup_logging():
    """Configures the logging for the application."""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

class StanleyController:
    """Calculates steering angle using the Stanley control method."""
    @staticmethod
    def calculate_steering(cross_track_error, heading_error, speed, k, epsilon, is_reverse):
        if is_reverse:
            cross_track_error = -cross_track_error
            cross_track_term = math.atan2(k * cross_track_error, speed + epsilon)
            return -(heading_error + cross_track_term)
        else:
            cross_track_term = math.atan2(k * cross_track_error, speed + epsilon)
            return heading_error + cross_track_term
    
    @staticmethod
    def calculate_velocity(cross_track_error, heading_error, speed, k_cte, k_heading, is_reverse):
        """Calculate the desired velocity based on cross track error and heading error."""
        # Reduce speed based on errors to maintain stability
        speed_factor = 1.0 / (1 + k_cte * abs(cross_track_error) + k_heading * abs(heading_error))
        desired_speed = speed * speed_factor
        
        if is_reverse:
            desired_speed *= -1
            
        return desired_speed

class LaneDetector:
    """Handles the full CV pipeline to detect lane and calculate errors using GPU."""
    def __init__(self, config: dict, use_cuda: bool):
        self.config = config
        self.ld_config = config['lane_detection']
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
        max_area = self.ld_config.get('max_contour_area', float('inf'))
        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > min_area and M['m00'] < max_area:
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
        pixels_per_meter = 1.0 / self.ld_config.get('pixels_to_meters', 0.0035) 
        axle_offset_m = self.ld_config.get('axle_to_bottom_frame_m', 0.0)
        
        y_offset_pixels = int(axle_offset_m * pixels_per_meter)
        virtual_axle_point = (W // 2, H - y_offset_pixels)
        
        if abs(m) < 1e-6:
            x_int, y_int = virtual_axle_point[0], b
        else:
            perp_m = -1 / m
            perp_b = virtual_axle_point[1] - perp_m * virtual_axle_point[0]
            x_int = (perp_b - b) / (m - perp_m)
            y_int = m * x_int + b
            
        intersection_point = (int(x_int), int(y_int))
        error_pixels = math.hypot(x_int - virtual_axle_point[0], y_int - virtual_axle_point[1])
        
        virtual_axle_y_coord = H - y_offset_pixels
        x_at_axle_line = (virtual_axle_y_coord - b) / m if abs(m) > 1e-6 else float('inf')
        
        if x_at_axle_line < W // 2:
            error_pixels = -error_pixels
            
        return error_pixels, intersection_point

    def process_frame(self, img):
        # [Process frame implementation remains the same]
        H, W = img.shape[:2]
        M = self._get_perspective_transform((H, W))

        if self.use_cuda:
            self.gpu_frame.upload(img)
            self.gpu_warped = cv2.cuda.warpPerspective(self.gpu_frame, M, (W, H), flags=cv2.INTER_LINEAR)
            self.gpu_hsv = cv2.cuda.cvtColor(self.gpu_warped, cv2.COLOR_BGR2HSV)
            self.gpu_mask = cv2.cuda.inRange(self.gpu_hsv, tuple(self.ld_config['hsv_lower']), tuple(self.ld_config['hsv_upper']))
            mask_yellow = self.gpu_mask.download()
            img_warped = self.gpu_warped.download()
        else:
            img_warped = cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_LINEAR)
            hsv_img = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(hsv_img, np.array(self.ld_config['hsv_lower']), np.array(self.ld_config['hsv_upper']))

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
    def draw_pipeline_data(self, image, data, steering_angle, current_speed, desired_speed):
        H, W = image.shape[:2]
        cv2.drawContours(image, data['contours'], -1, (255, 0, 255), 1)
        for center in data['centers']: 
            cv2.circle(image, (center['x'], center['y']), 5, (0, 255, 0), -1)
        if data['lane_points']: 
            cv2.line(image, data['lane_points'][0], data['lane_points'][1], (255, 0, 0), 3)
        if data['cte_intersection_point']:
            bottom_center = (W // 2, H)
            cv2.line(image, bottom_center, data['cte_intersection_point'], (255, 0, 255), 2)
        
        arrow_length = 100
        x0, y0 = W // 2, H
        arrow_angle = (math.pi / 2) - steering_angle
        x1, y1 = int(x0 + arrow_length * math.cos(arrow_angle)), int(y0 - arrow_length * math.sin(arrow_angle))
        cv2.arrowedLine(image, (x0, y0), (x1, y1), (0, 0, 255), 3, tipLength=0.2)
        
        cv2.putText(image, f"CTE: {data['cross_track_error']:.3f} m", (10, H - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"HE: {math.degrees(data['heading_error']):.1f} deg", (10, H - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"Steer: {math.degrees(steering_angle):.1f} deg", (10, H - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        cv2.putText(image, f"Speed: {current_speed:.1f} rpm", (10, H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(image, f"Target: {desired_speed:.1f} rpm", (10, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        return image

class LineFollowingNode(ManagedNode, ConfigMixin):
    """A managed node for running the line following vision pipeline."""
    
    def __init__(self, node_name: str = "line_follower_node", config_path: str = "config.yaml"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        self.camera = None
        self.camera_reverse = None
        self.lane_detector = None
        self.visualizer = None
        self.pub_socket = None
        self.hmi_sub_socket = None
        self.sensor_sub_socket = None
        self.result_writer = None
        self.is_reverse = False
        self.current_speed_rpm = 0.0  # Current vehicle speed from sensor data
        self.last_sensor_update = 0
        self.rpm_to_ms = 0.0  # Conversion factor from RPM to m/s
        self.vehicle_params = None
        
        self.processing_thread = None
        self.active_event = threading.Event()

    def on_configure(self) -> bool:
        """Load configuration and initialize all components."""
        self.logger.info("Configuring Line Following Node...")
        try:
            cam_config = self.get_camera_config('forward')  # Use forward camera config
            cam_config_reverse = self.get_camera_config('reverse')  # Use reverse camera config
            self.vehicle_params = self.get_section_config('vehicle_params')
            
            # Setup ZMQ Publisher for steering commands (now includes velocity)
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(self.get_zmq_url('steering_cmd_url'))
            
            # Setup ZMQ Subscriber for HMI direction commands
            self.hmi_sub_socket = self.context.socket(zmq.SUB)
            self.hmi_sub_socket.connect(self.get_zmq_url('hmi_cmd_url'))
            self.hmi_sub_socket.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('hmi_direction_topic'))

            # Setup ZMQ Subscriber for sensor data (speed feedback)
            self.sensor_sub_socket = self.context.socket(zmq.SUB)
            self.sensor_sub_socket.connect(self.get_zmq_url('sensor_data_url'))
            self.sensor_sub_socket.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('sensor_data_topic'))
            
            # Setup camera as ZMQ subscriber to camera frames
            zmq_camera_config = {
                'sub_frame_url': self.get_zmq_url('camera_frame_url'),
                'sub_frame_topic': self.get_zmq_topic('camera_frame_topic')
            }
            self.camera = ZMQCamera(cam_config, zmq_camera_config, self.context)

            zmq_camera_reverse_config = {
                'sub_frame_url': self.get_zmq_url('camera_frame_reverse_url'),
                'sub_frame_topic': self.get_zmq_topic('camera_frame_reverse_topic')
            }
            self.camera_reverse = ZMQCamera(cam_config, zmq_camera_reverse_config, self.context)

            # Setup lane detector
            use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
            self.logger.info(f"CUDA Available: {use_cuda}. {'Using GPU.' if use_cuda else 'Running on CPU.'}")
            self.lane_detector = LaneDetector(self.config, use_cuda)
            self.visualizer = Visualizer()
            
            # Setup video writer
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.result_writer = cv2.VideoWriter(
                cam_config['video_output_file'], fourcc, cam_config['frame_fps'],
                (cam_config['frame_width'] * 2, cam_config['frame_height'])
            )
            
            self.logger.info("Line Following Node configuration successful.")
            return True
            
        except Exception as e:
            self.logger.exception(f"Error during configuration: {e}")
            return False

    def on_activate(self) -> bool:
        """Start the camera and the processing loop."""
        self.logger.info("Activating Line Following Node...")
        try:
            self.camera.start()
            self.camera_reverse.start()
            self.active_event.set() 
            self.processing_thread = threading.Thread(
                target=self._processing_loop, 
                name=f"{self.node_name}_ProcessingThread"
            )
            self.processing_thread.start()
            self.logger.info("Line Following Node activated.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during activation: {e}")
            return False

    def on_deactivate(self) -> bool:
        """Stop the processing loop and camera."""
        self.logger.info("Deactivating Line Following Node...")
        try:
            self.active_event.clear()
            if self.processing_thread:
                self.processing_thread.join(timeout=2.0)
            if self.camera:
                self.camera.stop()
            if self.camera_reverse:
                self.camera_reverse.stop()
            self.logger.info("Line Following Node deactivated.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during deactivation: {e}")
            return False

    def on_shutdown(self) -> bool:
        """Clean up all resources."""
        self.logger.info("Shutting down Line Following Node...")
        if self.state == "active":
            self.on_deactivate()
            
        try:
            if self.result_writer:
                self.result_writer.release()
            if self.pub_socket:
                self.pub_socket.close()
            if self.hmi_sub_socket:
                self.hmi_sub_socket.close()
            if self.sensor_sub_socket:
                self.sensor_sub_socket.close()
            cv2.destroyAllWindows()
            self.logger.info("Line Following Node shutdown successful.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during shutdown: {e}")
            return False
            
    def _processing_loop(self):
        """Main processing loop for line following."""
        start_time = time.time()
        frame_count = 0
        cam_config = self.get_camera_config('forward')
        sc_config = self.get_section_config('stanley_controller')

        poller = zmq.Poller()
        poller.register(self.hmi_sub_socket, zmq.POLLIN)
        poller.register(self.sensor_sub_socket, zmq.POLLIN)
        if self.camera.socket:
            poller.register(self.camera.socket, zmq.POLLIN)
        if self.camera_reverse.socket:
            poller.register(self.camera_reverse.socket, zmq.POLLIN)

        while self.active_event.is_set() and not self.shutdown_event.is_set():
            # Check for HMI direction updates and sensor data
            socks = dict(poller.poll(100))
            if not socks:
                continue

            if self.hmi_sub_socket in socks:
                topic, msg = self.hmi_sub_socket.recv_multipart()
                self.logger.info(f"Received HMI command on topic {topic.decode('utf-8')}: {msg.decode('utf-8')}")
                if topic.decode('utf-8') == self.get_zmq_topic('hmi_direction_topic'):
                    self.logger.info(f"HMI command received: {msg.decode('utf-8')}")
                    self.is_reverse = msg.decode('utf-8') == 'reverse'

            # Update current speed from sensor data
            if self.sensor_sub_socket in socks:
                topic, sensor_data_json = self.sensor_sub_socket.recv_multipart()
                try:
                    import json
                    sensor_data = json.loads(sensor_data_json.decode('utf-8'))
                    self.current_speed_rpm = sensor_data.get('rpm', 0.0)
                    self.last_sensor_update = time.time()
                except (json.JSONDecodeError, KeyError) as e:
                    self.logger.warning(f"Failed to parse sensor data: {e}")
            
            # Check if sensor data is stale (use fallback speed)
            if time.time() - self.last_sensor_update > 1.0:  # 1 second timeout
                self.current_speed_rpm = sc_config.get('assumed_speed_for_calc', 500.0)
            
            # Process frame for lane detection
            rev_frame = self.camera_reverse.get_frame()
            forward_frame = self.camera.get_frame()
            if self.is_reverse:
                if rev_frame is None:
                    continue
                lane_data = self.lane_detector.process_frame(rev_frame)
            else:
                if forward_frame is None:
                    continue
                lane_data = self.lane_detector.process_frame(forward_frame)

            # Convert RPM to m/s for Stanley controller (approximate conversion)
            current_speed_ms = abs(self.current_speed_rpm) * self.vehicle_params['rpm_to_mps_factor']
            current_speed_kmh = current_speed_ms * 3.6

            # Calculate steering angle using Stanley controller with real speed
            steering_angle_rad = StanleyController.calculate_steering(
                lane_data['cross_track_error'],
                lane_data['heading_error'],
                0.5 if current_speed_ms < 1.0 else current_speed_ms,
                sc_config['gain'],
                sc_config['speed_epsilon'],
                self.is_reverse
            )
            steering_angle_deg = math.degrees(steering_angle_rad)

            # Calculate desired velocity
            desired_speed_ms = StanleyController.calculate_velocity(
                lane_data['cross_track_error'],
                lane_data['heading_error'],
                sc_config.get('max_speed', 1.0),
                sc_config.get('velocity_k_cte', 2.0),
                sc_config.get('velocity_k_heading', 1.0),
                self.is_reverse
            )

            desired_speed_rpm = desired_speed_ms / self.vehicle_params['rpm_to_mps_factor']

            # Send steering command and desired velocity via ZMQ
            command = steering_command_pb2.SteeringCommand()
            command.auto_steer_angle = steering_angle_deg
            command.speed = desired_speed_rpm  # Add this field to your protobuf
            serialized_command = command.SerializeToString()
            self.pub_socket.send_string(self.get_zmq_topic('steering_cmd_topic'), flags=zmq.SNDMORE)
            self.pub_socket.send(serialized_command)

            # Create visualization
            warped_view = lane_data.get('warped_image', np.zeros_like(forward_frame))
            annotated_warped = self.visualizer.draw_pipeline_data(
                warped_view, lane_data, steering_angle_rad, 
                self.current_speed_rpm, desired_speed_rpm
            )
            stacked_output = np.hstack((forward_frame, annotated_warped))
            
            # Display if enabled
            if cam_config.get('use_display', False):
                cv2.imshow('Lane Following View', stacked_output)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.logger.info("'q' pressed in display window, initiating shutdown.")
                    self.shutdown_event.set()
                    break
            
            # Write to video file
            self.result_writer.write(stacked_output)
            
            # Log performance metrics
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed_time = time.time() - start_time
                fps = frame_count / elapsed_time
                self.logger.info(f"Processing FPS: {fps:.2f}, Current Speed: {self.current_speed_rpm:.1f} RPM, Target: {desired_speed_rpm:.1f} RPM")
        
        self.logger.info("Line Following processing loop terminated.")

def main():
    setup_logging()
    line_follower = LineFollowingNode()
    line_follower.run()

if __name__ == "__main__":
    main()
