# frame_viewer_new.py
from datetime import datetime
import cv2
import yaml
import logging
import time

# --- MODIFIED: Import the new subscriber class ---
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))
from camera import FrameRecorder, ZMQFrameSubscriber 

def setup_logging():
    """Configures the logging for the application."""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

def load_config(path='../params/config.yaml'):
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

# --- REMOVED: The old zmq_subscriber_thread function is no longer needed ---
# The ZmqJpegSubscriberProcess class handles this logic internally.

def main():
    """
    Sets up the ZmqJpegSubscriberProcess to receive and display frames.
    """
    setup_logging()
    logger = logging.getLogger("FrameViewer")
    
    config = load_config()
    if not config:
        return

    # Use a default or configured IP
    PUBLISHER_IP = config.get('zmq', {}).get('publisher_ip', 'localhost')
    
    try:
        zmq_config = config.get('zmq', {})
        zmq_url_template = zmq_config.get('result_sub_url')
        zmq_topic = zmq_config.get('result_topic')

        if not zmq_url_template or not zmq_topic:
            logger.error("'result_sub_url' or 'camera_frame_topic' not found in config.yaml")
            return

        zmq_url = zmq_url_template.replace('*', PUBLISHER_IP)

    except KeyError as e:
        logger.error(f"Missing required configuration key: {e}")
        return

    # --- MODIFIED: Simplified Initialization ---
    # 1. Instantiate the subscriber class
    subscriber = ZMQFrameSubscriber(url=zmq_url, topic=zmq_topic)
    
    # 2. Start the background process
    subscriber.start()

    record_cfg = config.get('recording', {})
    frame_recorder = FrameRecorder(record_cfg)
    logger.info("Main thread started. Waiting for frames from subscriber process...")
    
    try:
        while True:
            # --- MODIFIED: Simplified Frame Retrieval ---
            # Get the latest decoded frame directly from the class.
            # This is a non-blocking call.
            frame = subscriber.get_latest()
            latency = subscriber.get_latency_ms()
            
            if frame is not None:
                
                if latency is not None:
                    latency_text = f"Latency: {latency:.1f} ms"
                    cv2.putText(frame, latency_text, (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                if record_cfg.get('enable', False):
                    if not frame_recorder.is_recording:
                        frame_recorder.start(frame.shape[1], frame.shape[0])
                    frame_recorder.write(frame)
                
                cv2.imshow("Remote Frame Viewer", frame)
            else:
                # If no frame is available yet, prevent the loop from spinning at 100% CPU
                # time.sleep(0.01)
                pass

            if cv2.waitKey(1) & 0xFF == ord('q'):
                logger.info("'q' pressed, shutting down.")
                break
                
    except KeyboardInterrupt:
        logger.info("Interrupted by user. Shutting down.")
    finally:
        # --- MODIFIED: Simplified Shutdown ---
        # The stop() method handles shutting down the process gracefully.
        logger.info("Signaling subscriber process to shut down...")
        subscriber.stop()
        cv2.destroyAllWindows()
        logger.info("Viewer shut down successfully.")

if __name__ == "__main__":
    main()