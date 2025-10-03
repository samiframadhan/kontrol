# image_viewer.py
import cv2
import zmq
import logging
import yaml
import threading
import queue
import time
import numpy as np  # <<< ADDED: For array reconstruction
import ast          # <<< ADDED: To safely evaluate the shape string

def setup_logging():
    """Configures the logging for the application."""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

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
    Receives multipart messages (topic, timestamp, shape, dtype, image_bytes)
    and puts the relevant data into a thread-safe queue for processing.
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
                # <<< MODIFIED: Receive five message parts now
                topic, timestamp_bytes, shape_bytes, dtype_bytes, image_bytes = socket.recv_multipart()
                
                try:
                    # <<< MODIFIED: Put a tuple with all necessary data into the queue
                    message_data = (timestamp_bytes, shape_bytes, dtype_bytes, image_bytes)
                    message_queue.put(message_data, block=False)
                except queue.Full:
                    try:
                        message_queue.get_nowait()
                        message_queue.put(message_data, block=False)
                    except queue.Empty:
                        pass
                    except queue.Full:
                        logger.warning("Queue is still full after dropping a frame.")

            except zmq.Again:
                continue
    finally:
        logger.info("Subscriber thread shutting down.")
        socket.close()
        context.term()

def find_lane_contours(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = 1200
    max_area = 50000
    lane_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area and cv2.contourArea(cnt) < max_area]
    return lane_contours

def pair_contours(contours):
    if len(contours) < 2:
        return None, None

    countours_and_centers_with_index = []
    for i, cnt in enumerate(contours):
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            countours_and_centers_with_index.append((cnt, (cX, cY), i))
    
    paired_contours = []
    used_indices = set()
    for i in range(len(countours_and_centers_with_index)):
        if i in used_indices:
            continue
        cnt1, center1, idx1 = countours_and_centers_with_index[i]
        best_match = None
        best_distance = float('inf')
        for j in range(i + 1, len(countours_and_centers_with_index)):
            if j in used_indices:
                continue
            cnt2, center2, idx2 = countours_and_centers_with_index[j]
            distance = np.linalg.norm(np.array(center1) - np.array(center2))
            if distance < best_distance:
                best_distance = distance
                best_match = (cnt2, center2, idx2)
        if best_match:
            paired_contours.append((cnt1, best_match[0]))
            used_indices.add(idx1)
            used_indices.add(best_match[2])

    return paired_contours

def main():
    """
    Reconstructs NumPy arrays from a queue and displays them as video frames.
    """
    setup_logging()
    logger = logging.getLogger("FrameViewer")
    datetime = time.strftime("%Y%m%d-%H%M%S")
    filename = 'output_' + datetime + '.avi'
    writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'XVID'), 20.0, (1280, 720))

    config = load_config()
    if not config:
        return

    try:
        zmq_config = config.get('zmq', {})
        zmq_url_template = zmq_config.get('camera_frame_url')
        zmq_topic = zmq_config.get('camera_frame_topic')
        publisher_ip = zmq_config.get('publisher_ip', 'localhost')

        if not zmq_url_template or not zmq_topic:
            logger.error("'camera_frame_url' or 'camera_frame_topic' not found in config.yaml")
            return

        zmq_url = zmq_url_template.replace('*', publisher_ip)

    except KeyError as e:
        logger.error(f"Missing required configuration key: {e}")
        return

    message_queue = queue.Queue(maxsize=2)
    shutdown_event = threading.Event()

    subscriber = threading.Thread(
        target=zmq_subscriber_thread,
        args=(zmq_url, zmq_topic, message_queue, shutdown_event),
        daemon=True
    )
    subscriber.start()
    
    logger.info("Main thread started. Waiting for messages from subscriber...")
    
    try:
        while True:
            try:
                # <<< MODIFIED: Get the 4-part tuple from the queue
                timestamp_bytes, shape_bytes, dtype_bytes, image_bytes = message_queue.get(timeout=1)
                
                # --- Latency Calculation (unchanged) ---
                recv_time = time.time_ns()
                send_time = int(timestamp_bytes.decode('utf-8'))
                latency_ms = (recv_time - send_time) / 1_000_000
                
                # <<< MODIFIED: Reconstruct the NumPy array from the received parts
                # 1. Evaluate the shape string into a tuple
                shape = ast.literal_eval(shape_bytes.decode('utf-8'))
                # 2. Get the data type
                dtype = np.dtype(dtype_bytes.decode('utf-8'))
                # 3. Create the NumPy array from the raw bytes, dtype, and shape
                frame = np.frombuffer(image_bytes, dtype=dtype).reshape(shape).copy()
                # Load camera calibration parameters
                calibration_fs = cv2.FileStorage('calibrationimx708hdr.yaml', cv2.FILE_STORAGE_READ)
                camera_matrix = calibration_fs.getNode('camera_matrix').mat()
                dist_coeffs = calibration_fs.getNode('distortion_coefficients').mat()
                calibration_fs.release()

                # Apply camera calibration
                frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray = cv2.GaussianBlur(gray, (5, 5), 0)
                white_mask = cv2.inRange(gray, 164, 185)
                kernel = np.ones((3,3), np.uint8)
                white_mask = cv2.erode(white_mask, kernel, iterations=2)
                white_mask = cv2.dilate(white_mask, kernel, iterations=2)
                lane_contours = find_lane_contours(white_mask)
                if lane_contours:
                    cv2.drawContours(frame, lane_contours, -1, (0, 255, 0), 3)
                    paired = pair_contours(lane_contours)
                    if paired:
                        for cnt1, cnt2 in paired:
                            M1 = cv2.moments(cnt1)
                            M2 = cv2.moments(cnt2)
                            if M1["m00"] != 0 and M2["m00"] != 0:
                                cX1 = int(M1["m10"] / M1["m00"])
                                cY1 = int(M1["m01"] / M1["m00"])
                                cX2 = int(M2["m10"] / M2["m00"])
                                cY2 = int(M2["m01"] / M2["m00"])
                                cv2.line(frame, (cX1, cY1), (cX2, cY2), (255, 0, 0), 2)
                                midX = (cX1 + cX2) // 2
                                midY = (cY1 + cY2) // 2
                                cv2.circle(frame, (midX, midY), 5, (0, 0, 255), -1)
                                frame_center_x = frame.shape[1] // 2
                                deviation = midX - frame_center_x
                                cv2.putText(frame, f"Deviation: {deviation}px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                writer.write(frame)
                
                # --- Display Logic (mostly unchanged) ---
                latency_text = f"Latency: {latency_ms:.1f} ms"
                cv2.putText(frame, latency_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                combined_frame = np.vstack([frame, cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)])
                cv2.imshow("Remote Frame Viewer", combined_frame)
                
            except queue.Empty:
                logger.info("Message queue is empty, waiting...")
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
        writer.release()
        cv2.destroyAllWindows()
        logger.info("Viewer shut down successfully.")

if __name__ == "__main__":
    main()
