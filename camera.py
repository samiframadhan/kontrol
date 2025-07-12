# zmq_camera_publisher.py
import cv2
import zmq
import numpy as np
import time
import logging
import yaml
from typing import Dict, Any

def setup_logging():
    """Configures basic logging for the application."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )

def load_config(config_path: str = 'camera.yaml') -> Dict[str, Any]:
    """
    Loads configuration from a YAML file.

    Args:
        config_path: Path to the YAML configuration file.

    Returns:
        A dictionary containing the configuration.
    """
    logging.info(f"Loading configuration from {config_path}...")
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        logging.info("Configuration loaded successfully.")
        return config
    except FileNotFoundError:
        logging.error(f"Configuration file not found at: {config_path}")
        exit(1)
    except Exception as e:
        logging.error(f"Error loading YAML configuration: {e}")
        exit(1)

def main():
    """
    Main function to capture frames from a camera or video file
    and publish them over a ZMQ PUB socket.
    """
    setup_logging()
    config = load_config()

    camera_config = config.get('camera', {})
    zmq_config = config.get('zmq', {})

    # --- ZMQ Setup ---
    context = zmq.Context()
    pub_socket = context.socket(zmq.PUB)
    try:
        pub_socket.bind(zmq_config['pub_frame_url'])
        logging.info(f"ZMQ Publisher bound to {zmq_config['pub_frame_url']}")
    except zmq.error.ZMQError as e:
        logging.error(f"Could not bind ZMQ socket: {e}. Is the address already in use?")
        context.term()
        return

    # --- OpenCV Camera Setup ---
    source = camera_config.get('source', 0) # Default to webcam 0
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        logging.error(f"Cannot open video source: {source}")
        pub_socket.close()
        context.term()
        return

    # Set camera properties from config
    frame_width = camera_config.get('frame_width', 640)
    frame_height = camera_config.get('frame_height', 480)
    fps = camera_config.get('frame_fps', 30)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    logging.info(f"Camera source '{source}' opened successfully.")
    logging.info(f"Publishing at approximately {fps} FPS.")
    logging.info("Press Ctrl+C to stop the publisher.")

    frame_count = 0
    start_time = time.time()
    sleep_interval = 1.0 / fps

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                logging.warning("End of video file or camera disconnected. Stopping.")
                break

            # Ensure the frame is the correct size, resizing if necessary
            # This is a fallback in case the cap.set() calls fail silently
            if frame.shape[1] != frame_width or frame.shape[0] != frame_height:
                 frame = cv2.resize(frame, (frame_width, frame_height))

            # Send the frame as a multipart message
            # 1. The topic (so subscribers can filter)
            # 2. The frame data as bytes
            pub_socket.send_string(zmq_config['frame_topic'], flags=zmq.SNDMORE)
            pub_socket.send(frame.tobytes())

            frame_count += 1
            logging.debug(f"Published frame {frame_count}")

            # Control the publishing rate
            time.sleep(sleep_interval)

            if frame_count % (fps * 5) == 0: # Log FPS every 5 seconds
                elapsed_time = time.time() - start_time
                current_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
                logging.info(f"Average publishing FPS: {current_fps:.2f}")

    except KeyboardInterrupt:
        logging.info("Ctrl+C detected. Shutting down.")
    except Exception as e:
        logging.exception("An unhandled error occurred in the main loop:")
    finally:
        # --- Cleanup ---
        logging.info("Cleaning up resources...")
        cap.release()
        pub_socket.close()
        context.term()
        logging.info("Shutdown complete.")

if __name__ == "__main__":
    main()
