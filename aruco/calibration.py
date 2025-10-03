# calibrate_from_stream.py
import cv2
import zmq
import logging
import yaml
import threading
import queue
import time
import numpy as np
import ast  # <<< ADDED
from cv2 import aruco
import argparse

def setup_logging():
    """Configures the logging for the application."""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

def load_config(path='../config.yaml'):
    """Loads the YAML configuration file."""
    try:
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        logging.warning(f"Configuration file not found at {path}. This is only required for ZMQ source.")
        return {}
    except yaml.YAMLError as e:
        logging.error(f"Error parsing YAML file: {e}")
        return None

def zmq_subscriber_thread(zmq_url, zmq_topic, message_queue, shutdown_event):
    """
    <<< MODIFIED: Receives multipart messages with NumPy array data.
    (topic, timestamp, shape, dtype, image_bytes)
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
                # <<< MODIFIED: Receive five message parts
                topic, timestamp_bytes, shape_bytes, dtype_bytes, image_bytes = socket.recv_multipart()
                
                # <<< MODIFIED: Put all necessary data into the queue
                message_data = (timestamp_bytes, shape_bytes, dtype_bytes, image_bytes)
                message_queue.put(message_data, block=False)
            except queue.Full:
                try:
                    message_queue.get_nowait()
                    message_queue.put(message_data, block=False)
                except (queue.Empty, queue.Full):
                    pass
            except zmq.Again:
                continue
    finally:
        logger.info("Subscriber thread shutting down.")
        socket.close()
        context.term()

def main():
    """
    Receives frames from a ZMQ stream or local camera and uses them for calibration.
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--captures", required=True, help="Number of captures required for calibration", type=int)
    ap.add_argument("-s", "--source", default="zmq", choices=["zmq", "local"], help="Source of the video stream ('zmq' or 'local')")
    args = vars(ap.parse_args())

    num_captures_required = args["captures"]
    video_source = args["source"]

    setup_logging()
    logger = logging.getLogger("Calibrator")

    message_queue = None
    shutdown_event = None
    subscriber = None
    local_cap = None

    if video_source == "zmq":
        config = load_config()
        if config is None: return
        try:
            zmq_config = config.get('zmq', {})
            zmq_url_template = zmq_config.get('camera_frame_url')
            zmq_topic = zmq_config.get('camera_frame_topic')
            publisher_ip = zmq_config.get('publisher_ip', 'localhost')
            if not zmq_url_template or not zmq_topic:
                logger.error("'camera_frame_url' or 'camera_frame_topic' not found in config.yaml.")
                return
            zmq_url = zmq_url_template.replace('*', publisher_ip)
        except KeyError as e:
            logger.error(f"Missing required configuration key for ZMQ: {e}")
            return

        message_queue = queue.Queue(maxsize=2)
        shutdown_event = threading.Event()
        subscriber = threading.Thread(
            target=zmq_subscriber_thread,
            args=(zmq_url, zmq_topic, message_queue, shutdown_event),
            daemon=True
        )
        subscriber.start()
    else: # video_source == "local"
        logger.info("Using local camera (ID: 0) as source.")
        local_cap = cv2.VideoCapture(0)
        if not local_cap.isOpened():
            logger.error("Could not open local camera.")
            return

    # --- ChArUco Board and Calibration State Setup ---
    CHARUCOBOARD_ROWCOUNT = 3
    CHARUCOBOARD_COLCOUNT = 7
    ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
    CHARUCO_BOARD = aruco.CharucoBoard(
        (CHARUCOBOARD_COLCOUNT, CHARUCOBOARD_ROWCOUNT),
        squareLength=0.030,
        markerLength=0.015,
        dictionary=ARUCO_DICT)

    corners_all = []
    ids_all = []
    image_size = None
    captures_count = 0

    logger.info(f"Main thread started. Using '{video_source}' source. Waiting for frames...")

    try:
        while True:
            frame = None
            latency_ms = None

            if video_source == "zmq":
                try:
                    # <<< MODIFIED: Unpack the 4-item tuple
                    timestamp_bytes, shape_bytes, dtype_bytes, image_bytes = message_queue.get(timeout=1)
                    
                    # <<< MODIFIED: Reconstruct the numpy array
                    shape = ast.literal_eval(shape_bytes.decode('utf-8'))
                    dtype = np.dtype(dtype_bytes.decode('utf-8'))
                    frame = np.frombuffer(image_bytes, dtype=dtype).reshape(shape).copy() # Added .copy()

                    recv_time = time.time_ns()
                    send_time = int(timestamp_bytes.decode('utf-8'))
                    latency_ms = (recv_time - send_time) / 1_000_000 # Convert ns to ms
                except queue.Empty:
                    logger.info("ZMQ message queue is empty, waiting...")
                    continue
            else: # video_source == "local"
                ret, frame = local_cap.read()
                if not ret:
                    logger.error("Failed to grab frame from local camera.")
                    break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=ARUCO_DICT)
            board_detected_this_frame = False

            if ids is not None:
                frame = aruco.drawDetectedMarkers(image=frame, corners=corners)
                response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    markerCorners=corners, markerIds=ids, image=gray, board=CHARUCO_BOARD)
                
                if response > 6: # Check for a reasonable number of detected corners
                    board_detected_this_frame = True
                    frame = aruco.drawDetectedCornersCharuco(image=frame, charucoCorners=charuco_corners, charucoIds=charuco_ids)
                    if not image_size:
                        image_size = gray.shape[::-1]

            # --- UI and Information Overlay ---
            if latency_ms is not None:
                cv2.putText(frame, f"Latency: {latency_ms:.1f} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(frame, f"Captures: {captures_count}/{num_captures_required}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if board_detected_this_frame:
                 cv2.putText(frame, "Board detected! Press 'c' to capture.", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)

            cv2.imshow("Calibration Stream", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("'q' pressed, shutting down.")
                break

            if key == ord('c') and board_detected_this_frame:
                corners_all.append(charuco_corners)
                ids_all.append(charuco_ids)
                captures_count += 1
                logger.info(f"Capture {captures_count}/{num_captures_required} saved.")
                time.sleep(0.5)

            if captures_count >= num_captures_required:
                logger.info("Sufficient captures collected. Starting calibration...")
                
                calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                    charucoCorners=corners_all,
                    charucoIds=ids_all,
                    board=CHARUCO_BOARD,
                    imageSize=image_size,
                    cameraMatrix=None,
                    distCoeffs=None)

                logger.info("Calibration successful!")
                print("\n--- Calibration Results ---")
                print(f"Reprojection Error: {calibration}")
                print(f"\nCamera Intrinsic Matrix:\n{cameraMatrix}")
                print(f"\nDistortion Coefficients:\n{distCoeffs}")

                fs = cv2.FileStorage('./CameraCalibration.yml', cv2.FILE_STORAGE_WRITE)
                fs.write("camera_matrix", cameraMatrix)
                fs.write("dist_coeff", distCoeffs)
                fs.release()
                logger.info('Calibration file saved: CameraCalibration.yml')
                break

    except KeyboardInterrupt:
        logger.info("Interrupted by user. Shutting down.")
    finally:
        if video_source == "zmq" and subscriber is not None:
            logger.info("Signaling subscriber thread to shut down...")
            shutdown_event.set()
            subscriber.join()
        if local_cap is not None:
            local_cap.release()
        cv2.destroyAllWindows()
        logger.info("Viewer shut down successfully.")

if __name__ == "__main__":
    main()
