import cv2
import datetime
import math
import numpy as np
import zmq
import time
import logging
import threading
import json
import multiprocessing
import simplejpeg as sjpg
from typing import List, Tuple, Optional

# Assuming these protobuf files are generated and available in the path
import steering_command_pb2
# import frame_data_pb2

# Assuming these are part of your project structure
from managednode import ManagedNode
from config_mixin import ConfigMixin
from camera import ZMQFrameSubscriber


def setup_logging():
    """Configures the logging for the application."""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(processName)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# --- Standalone Vision Processing Functions ---
# Preserved functions from linefollowing.py and new/adapted functions from tesstanley2.py

def _get_perspective_transform(ld_config, img_shape):
    """(Preserved) Calculates the perspective transform matrix."""
    H, W = img_shape
    warp = ld_config['perspective_warp']
    mirror_point = 1 - warp
    src = np.float32([[W * warp, H], [0, 0], [W, 0], [W * mirror_point, H]])
    dst = np.float32([[0, H], [0, 0], [W, 0], [W, H]])
    return cv2.getPerspectiveTransform(dst, src)

def _calculate_perpendicular_error(ld_config, m, b, H, W):
    """(Preserved) Calculates the cross-track error in pixels."""
    pixels_per_meter = 1.0 / ld_config.get('pixels_to_meters', 0.0035) 
    axle_offset_m = ld_config.get('axle_to_bottom_frame_m', 0.0)
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

# --- NEW Helper functions adapted from tesstanley2.py ---

def _contour_center(cnt: np.ndarray) -> Optional[Tuple[int, int]]:
    """(New) Calculates the center of a single contour."""
    M = cv2.moments(cnt)
    if M["m00"] == 0: return None
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

def _pair_midpoint(cnt1: np.ndarray, cnt2: np.ndarray) -> Optional[Tuple[int, int]]:
    """(New) Calculates the midpoint between the centers of two contours."""
    c1, c2 = _contour_center(cnt1), _contour_center(cnt2)
    if c1 is None or c2 is None: return None
    return (c1[0] + c2[0]) // 2, (c1[1] + c2[1]) // 2

def _find_marker_contours(ld_config: dict, mask: np.ndarray) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """(New) Finds contours that match marker criteria (area, shape)."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = ld_config.get('min_contour_area', 300)
    max_area = ld_config.get('max_contour_area', 3000)
    
    quad_contours = []
    size_threshold_contours = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area < area < max_area:
            size_threshold_contours.append(cnt)
            perimeter = cv2.arcLength(cnt, True)
            # Find contours with 4 vertices (quadrilaterals)
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
            if len(approx) == 4:
                quad_contours.append(cnt)

    return quad_contours, size_threshold_contours

def _pair_contours(ld_config: dict, contours: List[np.ndarray]) -> Optional[List[Tuple[np.ndarray, np.ndarray]]]:
    """(New) Pairs contours based on distance and area similarity."""
    if len(contours) < 2: return None

    entries = [(cnt, _contour_center(cnt), i) for i, cnt in enumerate(contours) if _contour_center(cnt) is not None]
    paired_contours, used_indices = [], set()

    min_dist = ld_config.get('min_pair_distance', 10)
    max_dist = ld_config.get('max_pair_distance', 100)
    max_area_diff = ld_config.get('max_pair_area_diff', 200)

    for i in range(len(entries)):
        if i in used_indices: continue
        cnt1, center1, idx1 = entries[i]
        best_match, best_distance = None, float('inf')

        for j in range(i + 1, len(entries)):
            if j in used_indices: continue
            cnt2, center2, idx2 = entries[j]
            distance = np.linalg.norm(np.array(center1) - np.array(center2))
            if distance < best_distance:
                best_distance, best_match = distance, (cnt2, center2, idx2)
        
        if best_match and (min_dist < best_distance < max_dist):
            area1, area2 = cv2.contourArea(cnt1), cv2.contourArea(best_match[0])
            if abs(area1 - area2) < max_area_diff:
                paired_contours.append((cnt1, best_match[0]))
                used_indices.add(idx1)
                used_indices.add(best_match[2])

    return paired_contours if paired_contours else None

# --- MODIFIED Core Vision Processing Function ---

def process_frame(frame_queue, result_queue, config):
    """
    A target function for a separate process that handles the CV pipeline.
    This version is CPU-only and uses the contour pairing method.
    """
    ld_config = config['lane_detection']
    logging.info("VisionProcess initialized for CPU processing with contour pairing.")

    while True:
        frame = frame_queue.get()
        if frame is None: # Sentinel value to stop the process
            break

        H, W = frame.shape[:2]
        M = _get_perspective_transform(ld_config, (H, W))

        # 1. Perspective Warp
        img_warped = cv2.warpPerspective(frame, M, (W, H), flags=cv2.INTER_LINEAR)
        
        # 2. Image Filtering and Thresholding (New Pipeline)
        filtered = cv2.bilateralFilter(img_warped, d=5, sigmaColor=175, sigmaSpace=175)
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
        # hsv = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV)
        # Using saturation channel, which is often robust to lighting changes
        # gray = hsv[:, :, 1] 
        binary_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 41, 2)
        
        # 3. Morphological Operations
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        eroded = cv2.erode(binary_mask, kernel, iterations=2)
        processed_mask = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # 4. Find, Pair Contours, and Find Midpoints
        quad_contours, all_contours = _find_marker_contours(ld_config, processed_mask)
        paired_contours = _pair_contours(ld_config, quad_contours)
        
        midpoints = [_pair_midpoint(p[0], p[1]) for p in paired_contours] if paired_contours else []
        valid_midpoints = [m for m in midpoints if m is not None]
        
        # Sort midpoints from bottom of the screen to the top
        midpoints_sorted = sorted(valid_midpoints, key=lambda p: -p[1])
        
        # Prepare data structure for downstream consumers
        pipeline_data = {
            'warped_image': img_warped, 
            'color_mask': processed_mask,  # Using the new processed mask
            'contours': all_contours,
            'paired_contours': paired_contours if paired_contours else [],
            'marker_midpoints': midpoints_sorted, 
            'cross_track_error': 0.0, 
            'heading_error': 0.0,
            'lane_points': [], 
            'cte_intersection_point': None
        }

        if len(midpoints_sorted) >= 2:
            # Use the two points closest to the vehicle as the reference line
            p1 = midpoints_sorted[0]
            p2 = midpoints_sorted[1]
            pipeline_data['lane_points'] = [p1, p2]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]

            if abs(dx) > 1e-6:
                m = dy / dx
                b = p1[1] - m * p1[0]
                # Heading error is the angle of the line relative to the vehicle's forward direction
                pipeline_data['heading_error'] = (math.pi / 2) - math.atan2(-dy, dx)
                
                # Use the preserved function to calculate perpendicular error
                error_pixels, intersection_point = _calculate_perpendicular_error(ld_config, m, b, H, W)
                pipeline_data['cross_track_error'] = error_pixels * ld_config['pixels_to_meters']
                pipeline_data['cte_intersection_point'] = intersection_point
        
        if not result_queue.full():
            result_queue.put(pipeline_data)

# --- Controller and Visualizer Classes ---

class StanleyController:
    """(Unchanged) Calculates steering angle using the Stanley control method."""
    @staticmethod
    def calculate_steering(cross_track_error, heading_error, speed, k, epsilon, is_reverse):
        cross_track_term = math.atan2(k * cross_track_error, abs(speed) + epsilon)
        return heading_error + cross_track_term
    
    @staticmethod
    def calculate_velocity(cross_track_error, heading_error, speed, k_cte, k_heading, is_reverse):
        """Calculate the desired velocity based on cross track error and heading error."""
        speed_factor = 1.0 / (1 + k_cte * abs(cross_track_error) + k_heading * abs(heading_error))
        desired_speed = speed * speed_factor
        if is_reverse:
            desired_speed *= -1
        return desired_speed

class Visualizer:
    """(Modified) Handles all OpenCV drawing operations to show new pipeline data."""
    def draw_pipeline_data(self, image, data, steering_angle, current_speed, desired_speed):
        H, W = image.shape[:2]
        # Draw all contours found after area filtering
        cv2.drawContours(image, data['contours'], -1, (255, 0, 255), 1)
        # Draw the paired contours in a bold color
        for pair in data.get('paired_contours', []):
            cv2.drawContours(image, list(pair), -1, (255, 255, 0), 2)
        # Draw the calculated midpoints
        for midpoint in data.get('marker_midpoints', []): 
            cv2.circle(image, midpoint, 5, (0, 0, 255), -1)
        # Draw the reference line for steering
        if data['lane_points']: 
            cv2.line(image, data['lane_points'][0], data['lane_points'][1], (255, 0, 0), 3)
        # Draw the cross-track error line
        if data['cte_intersection_point']:
            axle_point = (W // 2, H) # Simplified for visualization
            cv2.line(image, axle_point, data['cte_intersection_point'], (0, 255, 255), 2)
        
        # Steering and info display (unchanged)
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
    """(Unchanged) A managed node for running the line following vision pipeline."""
    
    def __init__(self, node_name: str = "line_follower_node", config_path: str = "config.yaml"):
        # This requires the ManagedNode and ConfigMixin classes to be in the project path
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        self.forward_frame_sub = None
        self.reverse_frame_sub = None
        
        self.frame_queue = None
        self.result_queue = None
        self.vision_process = None
        
        self.visualizer = None
        self.pub_socket = None
        self.hmi_sub_socket = None
        self.hmi_max_speed_sub = None
        self.sensor_sub_socket = None
        self.result_writer = None
        self.frame_pub_socket = None
        self.is_reverse = False
        self.current_speed_rpm = 0.0
        self.last_sensor_update = 0
        self.vehicle_params = None
        
        self.processing_thread = None
        self.active_event = threading.Event()
        self.current_max_speed = 0.0
        self.last_lane_data = None 

    def on_configure(self) -> bool:
        self.logger.info("Configuring Line Following Node (ZMQ camera)…")
        try:
            self.vehicle_params = self.get_section_config('vehicle_params')
            
            self.frame_queue = multiprocessing.Queue(maxsize=2)
            self.result_queue = multiprocessing.Queue(maxsize=2)
            
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(self.get_zmq_url('steering_cmd_url'))
            
            self.hmi_sub_socket = self.context.socket(zmq.SUB)
            self.hmi_sub_socket.connect(self.get_zmq_url('hmi_cmd_url'))
            self.hmi_sub_socket.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('hmi_direction_topic'))

            self.hmi_max_speed_sub = self.context.socket(zmq.SUB)
            self.hmi_max_speed_sub.connect(self.get_zmq_url('hmi_cmd_url'))
            self.hmi_max_speed_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('hmi_max_speed_topic'))

            self.sensor_sub_socket = self.context.socket(zmq.SUB)
            self.sensor_sub_socket.connect(self.get_zmq_url('sensor_data_url'))
            self.sensor_sub_socket.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('sensor_data_topic'))
            
            zmq_cfg = self.get_section_config('zmq')
            f_url = zmq_cfg.get('camera_frame_url')
            f_topic = zmq_cfg.get('camera_frame_topic')
            r_url = zmq_cfg.get('camera_frame_reverse_url')
            r_topic = zmq_cfg.get('camera_frame_reverse_topic')

            if not f_url or not f_topic:
                raise KeyError("Missing zmq.camera_frame_url or camera_frame_topic in config.yaml")
            if not r_url or not r_topic:
                self.logger.warning("Reverse camera ZMQ URL/topic not set; reverse mode will use forward stream.")
                r_url, r_topic = f_url, f_topic
            
            self.forward_frame_sub = ZMQFrameSubscriber(f_url, f_topic, name="forward_cam")
            self.reverse_frame_sub = ZMQFrameSubscriber(r_url, r_topic, name="reverse_cam")

            self.visualizer = Visualizer()

            if self.vehicle_params.get('enable_frame_publishing', False):
                self.frame_pub_socket = self.context.socket(zmq.PUB)
                publish_url = self.get_zmq_url('result_pub_url')
                self.frame_pub_socket.bind(publish_url)
                self.logger.info(f"Annotated frame publishing enabled. Broadcasting on {publish_url}")
            else:
                self.logger.info("Annotated frame publishing is disabled.")
            
            if self.vehicle_params.get('enable_video_recording', False):
                # Video recording setup...
                pass

            self.logger.info("Configuration successful.")
            return True
            
        except Exception as e:
            self.logger.exception(f"Error during configuration: {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating Line Following Node...")
        try:
            self.vision_process = multiprocessing.Process(
                target=process_frame,
                args=(self.frame_queue, self.result_queue, self.config),
                name="VisionProcess"
            )
            self.vision_process.start()
            
            self.forward_frame_sub.start()
            self.reverse_frame_sub.start()

            self.active_event.set() 
            self.processing_thread = threading.Thread(target=self._processing_loop, name=f"{self.node_name}_ProcessingThread")
            self.processing_thread.start()
            self.logger.info("Line Following Node activated.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during activation: {e}")
            return False

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating Line Following Node...")
        try:
            self.active_event.clear()
            
            if self.vision_process and self.vision_process.is_alive():
                self.frame_queue.put(None) 
                self.vision_process.join(timeout=2.0)
                if self.vision_process.is_alive():
                    self.vision_process.terminate()

            if self.processing_thread: self.processing_thread.join(timeout=2.0)
            if self.forward_frame_sub: self.forward_frame_sub.stop()
            if self.reverse_frame_sub: self.reverse_frame_sub.stop()

            self.logger.info("Line Following Node deactivated.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during deactivation: {e}")
            return False

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down Line Following Node...")
        if self.state == "active": self.on_deactivate()
        if self.frame_pub_socket:
            self.frame_pub_socket.close()
        if self.hmi_sub_socket:
            self.hmi_sub_socket.close()
        if self.hmi_max_speed_sub:
            self.hmi_max_speed_sub.close()
        if self.sensor_sub_socket:
            self.sensor_sub_socket.close()
        if self.pub_socket:
            self.pub_socket.close()
        return True
            
    def _processing_loop(self):
        """(Unchanged) Main processing loop for line following."""
        start_time = time.time()
        frame_count = 0
        sc_config = self.get_section_config('stanley_controller')
        
        # Poller setup... (omitted for brevity, remains unchanged)
        poller = zmq.Poller()
        poller.register(self.hmi_sub_socket, zmq.POLLIN)
        poller.register(self.sensor_sub_socket, zmq.POLLIN)
        poller.register(self.hmi_max_speed_sub, zmq.POLLIN)

        while self.active_event.is_set() and not self.shutdown_event.is_set():
            # ZMQ message polling... (omitted for brevity, remains unchanged)
            # Poll for ZMQ messages with a short timeout to not block frame handling
            socks = dict(poller.poll(timeout=5))

            if self.hmi_sub_socket in socks:
                topic, msg = self.hmi_sub_socket.recv_multipart()
                self.is_reverse = msg.decode('utf-8') == 'reverse'
                self.logger.info(f"Direction changed to {'REVERSE' if self.is_reverse else 'FORWARD'}")
            
            if self.hmi_max_speed_sub in socks:
                topic, msg = self.hmi_max_speed_sub.recv_multipart()
                try:
                    self.current_max_speed = float(msg.decode('utf-8'))
                    self.logger.info(f"Max speed updated to {self.current_max_speed} RPM.")
                except ValueError:
                    self.logger.warning(f"Invalid max speed value received: {msg.decode('utf-8')}")

            if self.sensor_sub_socket in socks:
                topic, sensor_data_json = self.sensor_sub_socket.recv_multipart()
                try:
                    sensor_data = json.loads(sensor_data_json.decode('utf-8'))
                    self.current_speed_rpm = sensor_data.get('rpm', 0.0)
                    self.last_sensor_update = time.time()
                except (json.JSONDecodeError, KeyError) as e:
                    self.logger.warning(f"Failed to parse sensor data: {e}")
            
            
            frame_source = self.reverse_frame_sub if self.is_reverse else self.forward_frame_sub
            frame = frame_source.get_latest()

            if frame is None:
                time.sleep(0.005)
                continue
            # frame = sjpg.decode_jpeg(frame, colorspace='BGR')
            
            try:
                self.frame_queue.put_nowait(frame)
            except multiprocessing.queues.Full:
                # self.logger.warning("Frame queue for vision process is full. Dropping frame.")
                pass

            try:
                lane_data = self.result_queue.get_nowait()
                self.last_lane_data = lane_data
            except Exception:
                if self.last_lane_data:
                    lane_data = self.last_lane_data
                else:
                    continue 

            current_speed_ms = abs(self.current_speed_rpm) * self.vehicle_params['rpm_to_mps_factor']
            
            desired_speed_ms = StanleyController.calculate_velocity(
                lane_data['cross_track_error'], lane_data['heading_error'],
                sc_config.get('max_speed', 1.5), sc_config.get('kcte', 2.0),
                sc_config.get('khe', 1.0), self.is_reverse
            )

            steering_angle_rad = StanleyController.calculate_steering(
                lane_data['cross_track_error'], lane_data['heading_error'],
                desired_speed_ms, sc_config['gain'], sc_config['speed_epsilon'], self.is_reverse
            )
            
            # Publishing and visualization logic (remains unchanged)
            # --- Publish Steering Command ---
            
            # 1. Convert calculated values to the required units (degrees and RPM)
            steering_angle_deg = math.degrees(steering_angle_rad)
            desired_speed_rpm = desired_speed_ms / self.vehicle_params['rpm_to_mps_factor']

            # In reverse, the Stanley controller's steering angle needs to be inverted
            if self.is_reverse:
                steering_angle_deg *= -1

            # 2. Send as multipart message (topic, angle, speed) as strings
            self.pub_socket.send_multipart([
                self.get_zmq_topic('steering_cmd_topic').encode('utf-8'),
                str(steering_angle_deg).encode('utf-8'),
                str(desired_speed_rpm).encode('utf-8')
            ])

            warped_view = lane_data.get('warped_image', np.zeros_like(frame))
            annotated_warped = self.visualizer.draw_pipeline_data(
                warped_view.copy(), lane_data, steering_angle_rad, 
                self.current_speed_rpm, 0 # Placeholder
            )
            stacked_output = np.hstack((frame, annotated_warped))

            if self.frame_pub_socket:
                try:
                    # Compress the frame using simplejpeg (fast)
                    # Quality can be adjusted (1-100) to trade quality for bandwidth
                    jpeg_bytes = sjpg.encode_jpeg(stacked_output, quality=85, colorspace='BGR')
                    
                    # Get current time in nanoseconds for latency calculation
                    timestamp = str(time.time_ns()).encode('utf-8')

                    # Publish as a multipart message [topic, timestamp, jpeg_bytes]
                    self.frame_pub_socket.send_multipart([
                        self.get_zmq_topic('result_topic').encode('utf-8'),
                        timestamp,
                        jpeg_bytes
                    ])
                except Exception as e:
                    self.logger.error(f"Failed to publish frame: {e}")
            
            if self.vehicle_params.get('use_display', False):
                cv2.imshow('Lane Following View', stacked_output)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.shutdown_event.set()
                    break
        
        self.logger.info("Line Following processing loop terminated.")

def main():
    setup_logging()
    # This example assumes ManagedNode, ConfigMixin, etc. are available.
    # If not, you may need to stub them for the script to run.
    # e.g., class ManagedNode: ...
    try:
        line_follower = LineFollowingNode(config_path="../params/config.yaml")
        line_follower.run()
    except (ImportError, FileNotFoundError) as e:
        logging.error(f"Could not start node. Missing dependencies or config: {e}")
        logging.error("Please ensure ManagedNode, ConfigMixin, and ZMQFrameSubscriber are in the Python path.")
        logging.error("Also ensure your config file path is correct.")


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()

