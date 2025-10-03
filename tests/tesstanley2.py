# tesdeteksi_modified.py
# MODIFIED: Adapted computer vision parameters to work with incoming 480p frames.
# MODIFIED: Added optional video recording of the processed stream.

import cv2
import zmq
import logging
import yaml
import multiprocessing as mp # MODIFIED: Replaced threading with multiprocessing
import simplejpeg as sjpg
import time
import numpy as np
import os
import math
from typing import List, Tuple, Optional
from datetime import datetime

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

def zmq_subscriber_process(zmq_url: str, zmq_topic: str, message_queue: mp.Queue, shutdown_event: mp.Event):
    """
    Receives ZMQ messages in a separate process and puts them into a thread-safe queue.
    This runs independently from the main processing loop to prevent I/O blocking.
    """
    # Each process should get its own logger instance.
    logger = logging.getLogger("ZMQSubscriberProcess")
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVTIMEO, 1000) # Timeout for recv
    logger.info(f"Connecting to publisher at {zmq_url}")
    socket.connect(zmq_url)
    socket.setsockopt_string(zmq.SUBSCRIBE, zmq_topic)
    logger.info(f"Subscriber process started. Subscribed to topic '{zmq_topic}'.")

    try:
        while not shutdown_event.is_set():
            try:
                # Receive message parts from ZMQ publisher
                topic, timestamp_bytes, msg_bytes = socket.recv_multipart()

                # Put the message into the shared queue for the main process
                try:
                    message_queue.put((timestamp_bytes, msg_bytes), block=False)
                except mp.queues.Full:
                    # If the queue is full, the main process is falling behind.
                    # We'll discard the oldest frame and add the new one.
                    try:
                        message_queue.get_nowait() # Remove oldest item
                        message_queue.put_nowait((timestamp_bytes, msg_bytes)) # Add newest item
                    except (mp.queues.Empty, mp.queues.Full):
                        # This can happen in a race condition, it's safe to ignore.
                        pass

            except zmq.Again:
                # This exception is expected on timeout, just continue the loop
                continue
    finally:
        logger.info("Subscriber process shutting down.")
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
    def calculate_velocity(cross_track_error, heading_error, max_speed, k_cte, k_heading, is_reverse):
        """Calculates the desired velocity based on errors."""
        speed_factor = 1.0 / (1 + k_cte * abs(cross_track_error) + k_heading * abs(heading_error))
        desired_speed = max_speed * speed_factor
        if is_reverse:
            desired_speed = -desired_speed
        return desired_speed

# =============================================================================
# 3. LANE DETECTION PIPELINE (MODIFIED)
# =============================================================================

# ---------------------- Contour helpers (MODIFIED) ----------------------
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
    # MODIFIED: Contour area values adjusted for 480p resolution (scaled down from 720p values)
    min_area, max_area = 250, 3000
    quad_contours = []
    size_threshold_contours = []
    print(f"Total contours found: {len(contours)}")
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area < area < max_area:
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
            print(f"Contour area: {area}, vertices: {len(approx)}")
            size_threshold_contours.append(cnt)
            if len(approx) == 4:
                quad_contours.append(cnt)
    print(f"Filtered quadrilateral contours: {len(quad_contours)}")
    return quad_contours, size_threshold_contours

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
        # MODIFIED: Distance and area difference thresholds adjusted for 480p resolution
        if best_match and (10 < best_distance < 100):
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
        if abs(m) < 1e-6:
            if p1[0] < W // 2:
                cte_px = -cte_px
        else:
            x_intercept_at_bottom = (H - b) / m
            if x_intercept_at_bottom < W // 2:
                cte_px = -cte_px
        control_data['cross_track_error_m'] = cte_px * vehicle_cfg['pixels_to_meters']
        x_closest = (vehicle_ref_pt[0] + m*vehicle_ref_pt[1] - m*b) / (m**2 + 1)
        y_closest = m * x_closest + b
        control_data['cte_line_pts'] = (vehicle_ref_pt, (int(x_closest), int(y_closest)))

    control_data['lane_line_pts'] = (p1, p2)
    
    control_data['desired_velocity'] = StanleyController.calculate_velocity(
        control_data['cross_track_error_m'], control_data['heading_error_rad'],
        stanley_cfg['max_speed'], stanley_cfg['kcte'], stanley_cfg['khe'], False
    )
    
    steering_rad = StanleyController.calculate_steering(
        control_data['cross_track_error_m'], control_data['heading_error_rad'],
        control_data['desired_velocity'], stanley_cfg['gain'], stanley_cfg['speed_epsilon']
    )
    control_data['steering_angle_deg'] = math.degrees(steering_rad)

    return control_data

# =============================================================================
# 4. LANE PROCESSING CLASS WITH TEMPORAL FILTERING (MODIFIED)
# =============================================================================
class LaneProcessor:
    def __init__(self, transform_cfg, vehicle_cfg, stanley_cfg):
        self.transform_cfg = transform_cfg
        self.vehicle_cfg = vehicle_cfg
        self.stanley_cfg = stanley_cfg

        self.last_valid_midpoint = None
        self.valid_frames_count = 0
        
        # MODIFIED: Pixel distance threshold adjusted for 480p resolution
        self.VALIDITY_THRESHOLD_PX = 150
        self.MIN_CONSECUTUTIVE_FRAMES = 2

    def reset_state(self):
        """Resets the tracking state."""
        self.last_valid_midpoint = None
        self.valid_frames_count = 0

    def process(self, frame: np.ndarray, camera_matrix, dist_coeffs):
        H, W = frame.shape[:2]
        # kernel = np.ones((3, 3), np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        
        M, src_poly = get_perspective_transform_from_warp_factor(H, W, self.transform_cfg['warp_factor'])
        warped_frame = cv2.warpPerspective(undistorted_frame, M, (W, H), flags=cv2.INTER_LINEAR)
        filtered = cv2.bilateralFilter(warped_frame, d=9, sigmaColor=75, sigmaSpace=75)
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (7, 7), 0)
        binary_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 19, 2)
        eroded = cv2.erode(binary_mask, kernel, iterations=1)
        processed_mask = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, kernel, iterations=1)
        # processed_mask = cv2.dilate(processed_mask, kernel, iterations=2)
        processed_mask = cv2.morphologyEx(processed_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        lane_contours, raw_contours = find_lane_contours(processed_mask)
        current_pairs = pair_contours(lane_contours) if lane_contours else []
        if current_pairs is None: current_pairs = []
        
        candidate_midpoints = [mid for cnt1, cnt2 in current_pairs if (mid := pair_midpoint(cnt1, cnt2)) is not None]
        best_match_midpoint = None
        
        if not candidate_midpoints:
            self.reset_state()
        else:
            if self.last_valid_midpoint is None:
                candidate_midpoints.sort(key=lambda p: abs(p[0] - W // 2))
                best_match_midpoint = candidate_midpoints[0]
                self.valid_frames_count = 1
            else:
                candidate_midpoints.sort(key=lambda p: np.linalg.norm(np.array(p) - np.array(self.last_valid_midpoint)))
                closest_candidate = candidate_midpoints[0]
                distance = np.linalg.norm(np.array(closest_candidate) - np.array(self.last_valid_midpoint))
                print(f"Distance to last valid midpoint: {distance:.2f} px")
                
                if distance < self.VALIDITY_THRESHOLD_PX:
                    self.valid_frames_count += 1
                    best_match_midpoint = closest_candidate
                else:
                    self.reset_state()
            
            self.last_valid_midpoint = best_match_midpoint

        is_lane_valid = self.valid_frames_count >= self.MIN_CONSECUTUTIVE_FRAMES
        
        midpoints_for_control = []
        if is_lane_valid and best_match_midpoint:
            for cnt1, cnt2 in current_pairs:
                mid = pair_midpoint(cnt1, cnt2)
                if mid == best_match_midpoint:
                    c1 = contour_center(cnt1)
                    c2 = contour_center(cnt2)
                    if c1 and c2:
                         midpoints_for_control = [c1, c2]
                    break

        control_data = calculate_errors_and_control(midpoints_for_control, H, W, self.vehicle_cfg, self.stanley_cfg)

        vis_warped = warped_frame.copy()
        if raw_contours:
            cv2.drawContours(vis_warped, raw_contours, -1, (0, 255, 255), 1)
        if lane_contours:
            cv2.drawContours(vis_warped, lane_contours, -1, (255, 0, 0), 2)
        line_color = (0, 255, 0) if is_lane_valid else (0, 165, 255)
        if control_data['lane_line_pts']:
            cv2.line(vis_warped, control_data['lane_line_pts'][0], control_data['lane_line_pts'][1], line_color, 3)
        if control_data['cte_line_pts']:
            cv2.line(vis_warped, control_data['cte_line_pts'][0], control_data['cte_line_pts'][1], (0, 255, 255), 2)
        for mid in candidate_midpoints:
            cv2.circle(vis_warped, mid, 5, (0, 0, 255), -1)

        # cv2.polylines(vis_warped, [src_poly.astype(np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.putText(vis_warped, f"CTE: {control_data['cross_track_error_m']:.3f} m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(vis_warped, f"HE: {math.degrees(control_data['heading_error_rad']):.1f} deg", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(vis_warped, f"Steer: {control_data['steering_angle_deg']:.1f} deg", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(vis_warped, f"Speed: {control_data['desired_velocity']:.2f} m/s", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        valid_text = f"Lane Valid: {is_lane_valid}"
        valid_color = (0, 255, 0) if is_lane_valid else (0, 0, 255)
        cv2.putText(vis_warped, valid_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, valid_color, 2)
        cv2.putText(vis_warped, f"Valid Count: {self.valid_frames_count}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        mask_bgr = cv2.cvtColor(processed_mask, cv2.COLOR_GRAY2BGR)
        combined_frame = np.hstack([vis_warped, mask_bgr])
        
        return combined_frame

# =============================================================================
# 5. FRAME RECORDER CLASS (NEW)
# =============================================================================
class FrameRecorder:
    """Handles optional video recording of processed frames."""
    def __init__(self, config: dict):
        self.is_enabled = config.get('enable', False)
        self.output_dir = config.get('output_dir', '../recordings')
        self.fps = config.get('fps', 30)
        self.writer = None
        self.is_recording = False
        self.frame_size = None  # (width, height)
        self.logger = logging.getLogger("FrameRecorder")

        if self.is_enabled:
            os.makedirs(self.output_dir, exist_ok=True)
            self.logger.info(f"Recording enabled. Output will be saved to '{self.output_dir}'")

    def start(self, frame_width: int, frame_height: int):
        """Starts a new recording session with a timestamped filename."""
        if not self.is_enabled or self.is_recording:
            return

        self.frame_size = (frame_width, frame_height)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"record_{timestamp}.mp4"
        filepath = os.path.join(self.output_dir, filename)

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        try:
            self.writer = cv2.VideoWriter(filepath, fourcc, self.fps, self.frame_size)
            if not self.writer.isOpened():
                self.logger.error(f"Failed to open VideoWriter for filepath: {filepath}")
                self.is_enabled = False  # Disable further attempts
                return
            self.is_recording = True
            self.logger.info(f"Started recording to {filepath}")
        except Exception as e:
            self.logger.error(f"Exception while creating VideoWriter: {e}")
            self.is_enabled = False  # Disable further attempts

    def write(self, frame: np.ndarray):
        """Writes a single frame to the video file."""
        if self.is_recording and self.writer is not None:
            self.writer.write(frame)

    def stop(self):
        """Stops the recording and releases the video writer."""
        if self.is_recording and self.writer is not None:
            self.writer.release()
            self.is_recording = False
            self.writer = None
            self.frame_size = None
            self.logger.info("Recording stopped and file saved.")

# =============================================================================
# 6. MAIN APPLICATION (MODIFIED)
# =============================================================================

def main():
    setup_logging()
    logger = logging.getLogger("MainApp")

    config = load_config(path="../params/config.yaml")
    if not config: return

    # --- Load Configuration (unchanged) ---
    try:
        zmq_config = config['zmq']
        zmq_url = zmq_config['camera_frame_url'].replace('*', zmq_config.get('publisher_ip', 'localhost'))
        zmq_topic = zmq_config['camera_frame_topic']
        transform_cfg = config['perspective_transform']
        vehicle_cfg = config['vehicle_params']
        stanley_cfg = config['stanley_controller']
        recording_cfg = config.get('recording', {})
        logger.info("Successfully loaded all configurations.")
    except KeyError as e:
        logger.error(f"Missing required configuration key/section: {e}. Please check config.yaml.")
        return

    # --- Load Camera Calibration (unchanged) ---
    try:
        calibration_file = '../params/calibrationimx708hdr.yaml'
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

    # --- Initialize Components ---
    lane_processor = LaneProcessor(transform_cfg, vehicle_cfg, stanley_cfg) # Unchanged
    recorder = FrameRecorder(recording_cfg) # Unchanged

    # MODIFIED: Use multiprocessing Queue and Event for inter-process communication.
    message_queue = mp.Queue(maxsize=2)
    shutdown_event = mp.Event()

    # MODIFIED: Instantiate and start the subscriber as a separate Process.
    subscriber = mp.Process(
        target=zmq_subscriber_process,
        args=(zmq_url, zmq_topic, message_queue, shutdown_event),
        daemon=True,  # Daemon process exits when the main program exits
        name="ZMQSubscriber"
    )
    subscriber.start()

    logger.info("Main process started. Waiting for messages from subscriber process...")

    try:
        while True:
            try:
                # Get the next frame from the queue filled by the subscriber process
                timestamp_bytes, jpeg_bytes = message_queue.get(timeout=1)
                frame = sjpg.decode_jpeg(jpeg_bytes, colorspace='BGR')

                # --- Core Processing Loop (unchanged) ---
                processed_frame = lane_processor.process(frame, camera_matrix, dist_coeffs)

                if recorder.is_enabled and not recorder.is_recording:
                    h, w, _ = processed_frame.shape
                    recorder.start(w, h)
                recorder.write(processed_frame)

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

            except mp.queues.Empty: # MODIFIED: Catch the correct queue empty exception
                logger.debug("Message queue is empty, main loop is waiting...")
                continue

            if cv2.waitKey(1) & 0xFF == ord('q'):
                logger.info("'q' pressed, shutting down.")
                break

    except KeyboardInterrupt:
        logger.info("Interrupted by user. Shutting down.")
    finally:
        logger.info("Initiating shutdown sequence...")

        # MODIFIED: Robust shutdown for the multiprocessing process.
        logger.info("Signaling subscriber process to shut down...")
        shutdown_event.set()
        subscriber.join(timeout=2)  # Wait up to 2 seconds for a graceful exit

        if subscriber.is_alive():
            logger.warning("Subscriber process did not exit gracefully. Terminating.")
            subscriber.terminate()  # Force terminate if it's stuck
            subscriber.join()       # Wait for termination to complete

        logger.info("Stopping recorder...")
        recorder.stop()
        cv2.destroyAllWindows()
        logger.info("Shutdown complete.")

if __name__ == "__main__":
    # This check is crucial for multiprocessing on some platforms (like Windows)
    # to prevent child processes from re-importing and re-executing the main script's code.
    main()