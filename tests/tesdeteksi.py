# integrated_stream_detector.py
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
from typing import List, Tuple, Optional, Set

# =============================================================================
# 1. SETUP & CONFIGURATION (from stream.py)
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
    """
    Receives multipart messages (topic, timestamp, jpeg_bytes) and puts
    the timestamp and jpeg_bytes into a thread-safe queue.
    """
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
                    pass # Ignore if queue is still full after trying to clear
            except zmq.Again:
                continue
    finally:
        logger.info("Subscriber thread shutting down.")
        socket.close()
        context.term()


# =============================================================================
# 2. LANE DETECTION PIPELINE (from tesdeteksi.py)
# =============================================================================

# ---------------------- Contour helpers ----------------------
def contour_center(cnt: np.ndarray) -> Optional[Tuple[int, int]]:
    """Return integer centroid (cX, cY) for a contour, or None if invalid."""
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

def pair_midpoint(cnt1: np.ndarray, cnt2: np.ndarray) -> Optional[Tuple[int, int]]:
    """Return the midpoint between the centroids of two contours, or None."""
    c1 = contour_center(cnt1)
    c2 = contour_center(cnt2)
    if c1 is None or c2 is None:
        return None
    (cX1, cY1), (cX2, cY2) = c1, c2
    return (cX1 + cX2) // 2, (cY1 + cY2) // 2

def find_lane_contours(mask: np.ndarray) -> List[np.ndarray]:
    """Finds, approximates, and filters contours for quadrilaterals."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area, max_area = 800, 5000
    quad_contours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area < area < max_area:
            perimeter = cv2.arcLength(cnt, True)
            epsilon = 0.04 * perimeter
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx) == 4:
                quad_contours.append(cnt)
    return quad_contours

def pair_contours(contours: List[np.ndarray]):
    """Pairs contours based on closest distance, with size and distance checks."""
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
        if best_match and (20 <= best_distance <= 100):
            area1, area2 = cv2.contourArea(cnt1), cv2.contourArea(best_match[0])
            if abs(area1 - area2) < 200:
                paired_contours.append((cnt1, best_match[0], abs(area1 - area2)))
                used_indices.add(idx1)
                used_indices.add(best_match[2])
    return paired_contours if paired_contours else None

# ---------------------- Matching: previous vs current ----------------------
def pairs_to_midpoints(pairs) -> List[Tuple[int, int]]:
    """Convert a list of contour pairs to a list of their midpoints."""
    return [mid for cnt1, cnt2, _ in pairs if (mid := pair_midpoint(cnt1, cnt2)) is not None] if pairs else []

def match_paired_contours(prev_pairs, curr_pairs, distance_thresh: float = 50.0):
    """Greedy nearest-neighbor matching between previous and current pair midpoints."""
    prev_mids, curr_mids = pairs_to_midpoints(prev_pairs), pairs_to_midpoints(curr_pairs)
    matches, valid_curr, used_prev = [], set(), set()
    for curr_idx, c_mid in enumerate(curr_mids):
        best_prev, best_dist = None, float('inf')
        for prev_idx, p_mid in enumerate(prev_mids):
            if prev_idx in used_prev: continue
            d = float(np.linalg.norm(np.array(c_mid) - np.array(p_mid)))
            if d < best_dist:
                best_dist, best_prev = d, prev_idx
        if best_prev is not None:
            matches.append((best_prev, curr_idx, best_dist))
            if best_dist <= distance_thresh:
                valid_curr.add(curr_idx)
                used_prev.add(best_prev)
    valid_distances = [d for _, _, d in matches if d <= distance_thresh]
    avg_distance = float(np.mean(valid_distances)) if valid_distances else float('nan')
    return matches, valid_curr, avg_distance

def compare_previous_pairing(prev_pairs, curr_pairs, distance_thresh: float = 60.0, min_match_ratio: float = 0.5) -> bool:
    """Check if the current pairing is consistent with the previous one."""
    if not prev_pairs or not curr_pairs: return False
    _, valid_curr, _ = match_paired_contours(prev_pairs, curr_pairs, distance_thresh)
    return len(valid_curr) / max(1, len(curr_pairs)) >= min_match_ratio

# ---------------------- Refactored Main Detection Function ----------------------
def process_frame(frame: np.ndarray, camera_matrix, dist_coeffs, previous_paired):
    """
    Applies the full lane detection pipeline to a single frame.

    Args:
        frame: The input image/frame (BGR).
        camera_matrix: The camera intrinsic matrix for undistortion.
        dist_coeffs: The distortion coefficients for undistortion.
        previous_paired: The contour pairs from the previous frame for comparison.

    Returns:
        A tuple containing:
        - processed_frame: The frame with detection overlays.
        - current_pairs: The contour pairs found in the current frame.
    """
    # 1. Undistort & preprocess
    undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
    gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
    binary_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                        cv2.THRESH_BINARY, 11, 2)
    kernel = np.ones((3, 3), np.uint8)
    processed_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # 2. Find and pair contours
    lane_contours = find_lane_contours(processed_mask)
    current_pairs = pair_contours(lane_contours) if lane_contours else None

    # 3. Compare with previous frame and draw overlays
    valid_idxs, avg_d = set(), float('nan')
    if previous_paired and current_pairs:
        if not compare_previous_pairing(previous_paired, current_pairs):
            logging.getLogger("Detection").info("Contour pairing changed significantly.")
        _, valid_idxs, avg_d = match_paired_contours(previous_paired, current_pairs)

    if current_pairs:
        frame_center_x = undistorted_frame.shape[1] // 2
        for idx, (cnt1, cnt2, size_diff) in enumerate(current_pairs):
            c1, c2 = contour_center(cnt1), contour_center(cnt2)
            if c1 is None or c2 is None: continue
            
            mid = pair_midpoint(cnt1, cnt2)
            if mid is None: continue
            midX, midY = mid
            
            is_valid = idx in valid_idxs
            line_color = (255, 0, 0) if is_valid else (128, 128, 128) # Blue / Gray
            dot_color = (0, 0, 255) if is_valid else (160, 160, 160)   # Red / Gray

            cv2.line(undistorted_frame, c1, c2, line_color, 2)
            cv2.circle(undistorted_frame, (midX, midY), 5, dot_color, -1)
            
            deviation = midX - frame_center_x
            cv2.putText(undistorted_frame, f"Dev:{deviation:+d} dA:{int(size_diff)}", (midX + 8, midY - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

        # HUD
        valid_count, total_pairs = len(valid_idxs), len(current_pairs)
        cv2.putText(undistorted_frame, f"Valid pairs: {valid_count}/{total_pairs} | avgDist: {avg_d:.1f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    
    combined_frame = np.vstack([undistorted_frame, cv2.cvtColor(processed_mask, cv2.COLOR_GRAY2BGR)])

    return combined_frame, current_pairs


# =============================================================================
# 3. MAIN APPLICATION (Integration of stream.py and tesdeteksi.py)
# =============================================================================

def main():
    """
    Subscribes to a ZMQ stream, runs lane detection on each frame,
    and displays the result with latency info.
    """
    setup_logging()
    logger = logging.getLogger("MainApp")
    
    # --- Load ZMQ Config ---
    config = load_config()
    if not config: return
    try:
        zmq_config = config.get('zmq', {})
        zmq_url = zmq_config.get('camera_frame_url').replace('*', zmq_config.get('publisher_ip', 'localhost'))
        zmq_topic = zmq_config.get('camera_frame_topic')
        if not zmq_url or not zmq_topic:
            raise KeyError("'camera_frame_url' or 'camera_frame_topic' missing")
    except (KeyError, AttributeError) as e:
        logger.error(f"Missing required configuration key: {e}")
        return

    # --- Load Camera Calibration ---
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

    # --- Start ZMQ Subscriber Thread ---
    message_queue = queue.Queue(maxsize=2)
    shutdown_event = threading.Event()
    subscriber = threading.Thread(
        target=zmq_subscriber_thread,
        args=(zmq_url, zmq_topic, message_queue, shutdown_event),
        daemon=True
    )
    subscriber.start()
    
    logger.info("Main thread started. Waiting for messages...")
    
    # --- Initialize Detection State ---
    previous_paired_contours = None

    try:
        while True:
            try:
                timestamp_bytes, jpeg_bytes = message_queue.get(timeout=1)
                
                # Decode frame
                frame = sjpg.decode_jpeg(jpeg_bytes, colorspace='BGR')

                # <<< INTEGRATION POINT >>>
                # Run the detection pipeline on the new frame
                processed_frame, current_pairs = process_frame(
                    frame, camera_matrix, dist_coeffs, previous_paired_contours
                )
                # Update state for the next iteration
                previous_paired_contours = current_pairs
                
                # Calculate and display latency
                recv_time_ns = time.time_ns()
                send_time_ns = int(timestamp_bytes.decode('utf-8'))
                latency_ms = (recv_time_ns - send_time_ns) / 1_000_000_000
                
                latency_text = f"Latency: {latency_ms:.1f} ms"
                cv2.putText(processed_frame, latency_text, (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                cv2.imshow("Lane Detection Stream", processed_frame)
                
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
