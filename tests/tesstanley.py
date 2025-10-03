# tesdeteksi_modified.py
import cv2
import zmq
import logging
import yaml
import threading
import queue
import simplejpeg as sjpg
import time
import numpy as np
import os
import math
from typing import List, Tuple, Optional

# =============================================================================
# 1. SETUP & CONFIGURATION (Unaltered)
# =============================================================================

def setup_logging():
    """Configures the logging for the application."""
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')

def load_config(path='config.yaml'):
    """Loads the YAML configuration file."""
    try:
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        logging.error(f"Configuration file not found at {path}")
        return None
    except yaml.YAMLError as e:
        logging.error(f"Error parsing YAML file: {e}")
        return None

def zmq_subscriber_thread(zmq_url, zmq_topic, message_queue, shutdown_event):
    """Receives ZMQ messages and puts them into a thread-safe queue."""
    logger = logging.getLogger("ZMQSubscriberThread")
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVTIMEO, 1000)
    logger.info(f"Connecting to publisher at {zmq_url}")
    socket.connect(zmq_url)
    socket.setsockopt_string(zmq.SUBSCRIBE, zmq_topic)
    logger.info(f"Subscriber thread started. Subscribed to topic '{zmq_topic}'.")
    try:
        while not shutdown_event.is_set():
            try:
                topic, timestamp_bytes, msg_bytes = socket.recv_multipart()
                message_queue.put((timestamp_bytes, msg_bytes), block=False)
            except queue.Full:
                try:
                    message_queue.get_nowait()
                    message_queue.put((timestamp_bytes, msg_bytes), block=False)
                except (queue.Empty, queue.Full):
                    pass
            except zmq.Again:
                continue
    finally:
        logger.info("Subscriber thread shutting down.")
        socket.close()
        context.term()

# =============================================================================
# 2. STANLEY CONTROLLER (Unaltered)
# =============================================================================

class StanleyController:
    """Calculates steering angle and velocity using Stanley control."""
    @staticmethod
    def calculate_steering(cross_track_error, heading_error, speed, k, epsilon):
        """Calculates the steering angle in radians."""
        cross_track_term = math.atan2(k * cross_track_error, abs(speed) + epsilon)
        return heading_error + cross_track_term

    @staticmethod
    def calculate_velocity(cross_track_error, heading_error, max_speed, k_cte, k_heading):
        """Calculates the desired velocity based on errors."""
        speed_factor = 1.0 / (1 + k_cte * abs(cross_track_error) + k_heading * abs(heading_error))
        return max_speed * speed_factor

# =============================================================================
# 3. LANE DETECTION PIPELINE (MODIFIED)
# =============================================================================

# ---------------------- Contour helpers (Unaltered) ----------------------
def contour_center(cnt: np.ndarray) -> Optional[Tuple[int, int]]:
    M = cv2.moments(cnt)
    if M["m00"] == 0: return None
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

def pair_midpoint(cnt1: np.ndarray, cnt2: np.ndarray) -> Optional[Tuple[int, int]]:
    c1, c2 = contour_center(cnt1), contour_center(cnt2)
    if c1 is None or c2 is None: return None
    return (c1[0] + c2[0]) // 2, (c1[1] + c2[1]) // 2

def find_lane_contours(mask: np.ndarray) -> List[np.ndarray]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area, max_area = 800, 5000
    quad_contours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area < area < max_area:
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * perimeter, True)
            if len(approx) == 4:
                quad_contours.append(cnt)
    return quad_contours

def get_perspective_transform_from_warp_factor(H: int, W: int, warp_factor: float):
    if not (0.0 <= warp_factor < 0.5):
        raise ValueError("warp_factor must be between 0.0 and 0.5")
    src_points = np.float32([
        [0, 0],
        [W, 0],
        [W * (1 - warp_factor), H],
        [W * warp_factor, H]
    ])
    dst_points = np.float32([
        [0, 0],
        [W, 0],
        [W, H],
        [0, H]
    ])
    M = cv2.getPerspectiveTransform(dst_points, src_points)
    return M, src_points

def pair_contours(contours: List[np.ndarray]):
    if len(contours) < 2: return None
    entries = [(cnt, contour_center(cnt), i) for i, cnt in enumerate(contours) if contour_center(cnt) is not None]
    paired_contours, used_indices = [], set()
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
        if best_match and (10 < best_distance < 150):
            area1, area2 = cv2.contourArea(cnt1), cv2.contourArea(best_match[0])
            if abs(area1 - area2) < 200:
                paired_contours.append((cnt1, best_match[0]))
                used_indices.add(idx1)
                used_indices.add(best_match[2])
    return paired_contours if paired_contours else None

# ---------------------- Error and Control Calculation (Unaltered) ----------------------
def calculate_errors_and_control(paired_midpoints: List[Tuple[int, int]], H: int, W: int, vehicle_cfg: dict, stanley_cfg: dict):
    control_data = {
        'cross_track_error_m': 0.0, 'heading_error_rad': 0.0,
        'steering_angle_deg': 0.0, 'desired_velocity': 0.0,
        'lane_line_pts': None, 'cte_line_pts': None
    }
    
    if len(paired_midpoints) < 2:
        return control_data

    midpoints_sorted = sorted(paired_midpoints, key=lambda p: -p[1])
    p1, p2 = midpoints_sorted[0], midpoints_sorted[1]
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]

    control_data['heading_error_rad'] = (math.pi / 2) - math.atan2(-dy, dx)
    
    if abs(dx) > 1e-6:
        m = dy / dx
        b = p1[1] - m * p1[0]
        vehicle_ref_pt = (W // 2, H)
        cte_px = abs(m * vehicle_ref_pt[0] - vehicle_ref_pt[1] + b) / math.sqrt(m**2 + 1)
        # --- START: MODIFIED BLOCK ---
        # Check for horizontal line to avoid division by zero
        if abs(m) < 1e-6:
            # For a horizontal line, check the x-position of the points directly
            # to determine if the lane is to the left or right of the center.
            if p1[0] < W // 2:
                cte_px = -cte_px
        else:
            # Original logic for non-horizontal lines
            x_intercept_at_bottom = (H - b) / m
            if x_intercept_at_bottom < W // 2:
                cte_px = -cte_px
        # --- END: MODIFIED BLOCK ---
        control_data['cross_track_error_m'] = cte_px * vehicle_cfg['pixels_to_meters']
        x_closest = (vehicle_ref_pt[0] + m*vehicle_ref_pt[1] - m*b) / (m**2 + 1)
        y_closest = m * x_closest + b
        control_data['cte_line_pts'] = (vehicle_ref_pt, (int(x_closest), int(y_closest)))

    control_data['lane_line_pts'] = (p1, p2)
    
    control_data['desired_velocity'] = StanleyController.calculate_velocity(
        control_data['cross_track_error_m'], control_data['heading_error_rad'],
        stanley_cfg['max_speed'], stanley_cfg['kcte'], stanley_cfg['khe']
    )
    
    steering_rad = StanleyController.calculate_steering(
        control_data['cross_track_error_m'], control_data['heading_error_rad'],
        control_data['desired_velocity'], stanley_cfg['gain'], stanley_cfg['speed_epsilon']
    )
    control_data['steering_angle_deg'] = math.degrees(steering_rad)

    return control_data

# =============================================================================
# 4. <<< NEW >>> LANE PROCESSING CLASS WITH TEMPORAL FILTERING
# =============================================================================
class LaneProcessor:
    def __init__(self, transform_cfg, vehicle_cfg, stanley_cfg):
        self.transform_cfg = transform_cfg
        self.vehicle_cfg = vehicle_cfg
        self.stanley_cfg = stanley_cfg

        # State for temporal filtering
        self.last_valid_midpoint = None
        self.valid_frames_count = 0
        
        # Configuration for the filter
        self.VALIDITY_THRESHOLD_PX = 30  # Max distance in pixels for a pair to be considered the same as the last frame
        self.MIN_CONSECUTIVE_FRAMES = 3  # Required consecutive frames to consider the lane "valid"

    def reset_state(self):
        """Resets the tracking state."""
        self.last_valid_midpoint = None
        self.valid_frames_count = 0

    def process(self, frame: np.ndarray, camera_matrix, dist_coeffs):
        H, W = frame.shape[:2]
        
        # 1. Undistort
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        # 2. Bird's-Eye View
        M, src_poly = get_perspective_transform_from_warp_factor(H, W, self.transform_cfg['warp_factor'])
        warped_frame = cv2.warpPerspective(undistorted_frame, M, (W, H), flags=cv2.INTER_LINEAR)

        # 3. Preprocess
        gray = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2GRAY)
        binary_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        kernel = np.ones((5, 5), np.uint8)
        processed_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel, iterations=1)

        # 4. Find and pair contours
        lane_contours = find_lane_contours(processed_mask)
        current_pairs = pair_contours(lane_contours) if lane_contours else []
        if current_pairs is None: current_pairs = []
        
        # 5. <<< MODIFIED >>> Filter and validate the detected pairs
        
        # Get all potential midpoints from the current frame's pairs
        candidate_midpoints = [mid for cnt1, cnt2 in current_pairs if (mid := pair_midpoint(cnt1, cnt2)) is not None]
        
        best_match_midpoint = None
        
        if not candidate_midpoints:
            self.reset_state()
        else:
            if self.last_valid_midpoint is None:
                # BOOTSTRAP: No prior valid frame, find the candidate pair closest to the center
                candidate_midpoints.sort(key=lambda p: abs(p[0] - W // 2))
                best_match_midpoint = candidate_midpoints[0]
                self.valid_frames_count = 1
            else:
                # TRACKING: Find the candidate closest to the last known valid midpoint
                candidate_midpoints.sort(key=lambda p: np.linalg.norm(np.array(p) - np.array(self.last_valid_midpoint)))
                closest_candidate = candidate_midpoints[0]
                distance = np.linalg.norm(np.array(closest_candidate) - np.array(self.last_valid_midpoint))
                
                if distance < self.VALIDITY_THRESHOLD_PX:
                    # Match is close enough, it's likely the same lane
                    self.valid_frames_count += 1
                    best_match_midpoint = closest_candidate
                else:
                    # Match is too far, lost the lock
                    self.reset_state()
            
            self.last_valid_midpoint = best_match_midpoint

        # 6. <<< MODIFIED >>> Calculate controls ONLY if the lane is valid
        is_lane_valid = self.valid_frames_count >= self.MIN_CONSECUTIVE_FRAMES
        
        midpoints_for_control = []
        if is_lane_valid and best_match_midpoint:
            # We need at least two points for error calculation. We'll use the best match
            # and extrapolate a second point to define the line. For simplicity, we can
            # just pass the midpoints from the single valid pair.
            # Find the original pair that corresponds to the best_match_midpoint
            for cnt1, cnt2 in current_pairs:
                mid = pair_midpoint(cnt1, cnt2)
                if mid == best_match_midpoint:
                    # Found the pair. Get the midpoints of the individual contours.
                    c1 = contour_center(cnt1)
                    c2 = contour_center(cnt2)
                    if c1 and c2:
                         midpoints_for_control = [c1, c2]
                    break

        control_data = calculate_errors_and_control(midpoints_for_control, H, W, self.vehicle_cfg, self.stanley_cfg)

        # 7. Visualization
        vis_warped = warped_frame.copy()
        line_color = (0, 255, 0) if is_lane_valid else (0, 165, 255) # Green if valid, Orange if candidate
        if control_data['lane_line_pts']:
            cv2.line(vis_warped, control_data['lane_line_pts'][0], control_data['lane_line_pts'][1], line_color, 3)
        if control_data['cte_line_pts']:
            cv2.line(vis_warped, control_data['cte_line_pts'][0], control_data['cte_line_pts'][1], (0, 255, 255), 2)
        for mid in candidate_midpoints:
            cv2.circle(vis_warped, mid, 5, (0, 0, 255), -1) # All candidates are red dots

        # 8. Create combined view with HUD
        cv2.polylines(undistorted_frame, [src_poly.astype(np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
        
        # Display HUD
        cv2.putText(undistorted_frame, f"CTE: {control_data['cross_track_error_m']:.3f} m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(undistorted_frame, f"HE: {math.degrees(control_data['heading_error_rad']):.1f} deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(undistorted_frame, f"Steer: {control_data['steering_angle_deg']:.1f} deg", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(undistorted_frame, f"Speed: {control_data['desired_velocity']:.2f} m/s", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # <<< NEW HUD INFO >>>
        valid_text = f"Lane Valid: {is_lane_valid}"
        valid_color = (0, 255, 0) if is_lane_valid else (0, 0, 255)
        cv2.putText(undistorted_frame, valid_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, valid_color, 2)
        cv2.putText(undistorted_frame, f"Valid Count: {self.valid_frames_count}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        mask_bgr = cv2.cvtColor(processed_mask, cv2.COLOR_GRAY2BGR)
        combined_frame = np.hstack([undistorted_frame, vis_warped, mask_bgr])
        
        return combined_frame

# =============================================================================
# 5. MAIN APPLICATION (MODIFIED)
# =============================================================================

def main():
    setup_logging()
    logger = logging.getLogger("MainApp")
    
    config = load_config()
    if not config: return
    
    try:
        zmq_config = config['zmq']
        zmq_url = zmq_config['camera_frame_url'].replace('*', zmq_config.get('publisher_ip', 'localhost'))
        zmq_topic = zmq_config['camera_frame_topic']
    except KeyError as e:
        logger.error(f"Missing required ZMQ configuration key: {e}")
        return

    try:
        transform_cfg = config['perspective_transform']
        vehicle_cfg = config['vehicle_params']
        stanley_cfg = config['stanley_controller']
        logger.info("Successfully loaded transform, vehicle, and controller configs.")
    except KeyError as e:
        logger.error(f"Missing required configuration section: {e}. Please check config.yaml.")
        return

    try:
        calibration_file = 'calibrationimx708hdr.yaml'
        if not os.path.exists(calibration_file):
            raise FileNotFoundError(f"Calibration file not found at '{calibration_file}'")
        fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        camera_matrix = fs.getNode('camera_matrix').mat()
        dist_coeffs = fs.getNode('distortion_coefficients').mat()
        fs.release()
        logger.info("Successfully loaded camera calibration parameters.")
    except Exception as e:
        logger.error(f"Could not load camera calibration: {e}. Cannot proceed.")
        return
        
    # <<< MODIFIED >>> Instantiate the LaneProcessor
    lane_processor = LaneProcessor(transform_cfg, vehicle_cfg, stanley_cfg)

    message_queue = queue.Queue(maxsize=2)
    shutdown_event = threading.Event()
    subscriber = threading.Thread(target=zmq_subscriber_thread, args=(zmq_url, zmq_topic, message_queue, shutdown_event), daemon=True)
    subscriber.start()
    
    logger.info("Main thread started. Waiting for messages...")

    try:
        while True:
            try:
                timestamp_bytes, jpeg_bytes = message_queue.get(timeout=1)
                frame = sjpg.decode_jpeg(jpeg_bytes, colorspace='BGR')

                # <<< MODIFIED >>> Use the processor object to process the frame
                processed_frame = lane_processor.process(frame, camera_matrix, dist_coeffs)
                
                recv_time_ns = time.time_ns()
                send_time_ns = int(timestamp_bytes.decode('utf-8'))
                latency_ms = (recv_time_ns - send_time_ns) / 1_000_000
                
                latency_text = f"Latency: {latency_ms:.1f} ms"
                cv2.putText(processed_frame, latency_text, (processed_frame.shape[1] - 250, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                display_height = 480
                h, w, _ = processed_frame.shape
                aspect_ratio = w / h
                display_width = int(display_height * aspect_ratio)
                display_frame = cv2.resize(processed_frame, (display_width, display_height))

                cv2.imshow("Lane Detection Stream", display_frame)
                
            except queue.Empty:
                logger.debug("Message queue is empty, waiting...")
                continue
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logger.info("'q' pressed, shutting down.")
                break
                
    except KeyboardInterrupt:
        logger.info("Interrupted by user. Shutting down.")
    finally:
        logger.info("Signaling subscriber thread to shut down...")
        shutdown_event.set()
        subscriber.join()
        cv2.destroyAllWindows()
        logger.info("Viewer shut down successfully.")

if __name__ == "__main__":
    main()