# camera_publisher.py
import cv2
import zmq
import numpy as np
import logging
import yaml
import time
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import frame_data_pb2 # This is the generated file from your .proto

def setup_logging():
    """Configures the logging for the application."""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

def load_config(path='../config.yaml'):
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

def main():
    """
    Captures frames from a camera, serializes them using Protobuf,
    and publishes them over a ZMQ PUB socket.
    """
    setup_logging()
    logger = logging.getLogger("CameraPublisher")

    # --- Configuration ---
    config = load_config()
    if not config:
        return

    try:
        zmq_config = config.get('zmq', {})
        zmq_url = zmq_config.get('camera_frame_url')
        zmq_topic_str = zmq_config.get('camera_frame_topic')
        
        if not zmq_url or not zmq_topic_str:
            logger.error("'camera_frame_url' or 'frame_publish_topic' not found in config.yaml")
            return

        # ZMQ topics need to be bytes
        zmq_topic = zmq_topic_str.encode('utf-8')

    except KeyError as e:
        logger.error(f"Missing required configuration key: {e}")
        return

    # --- ZMQ Setup ---
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    logger.info(f"Binding publisher to {zmq_url}")
    socket.bind(zmq_url)

    # --- OpenCV Camera Setup ---
    # Use camera index 0 (the default webcam)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logger.error("Error: Could not open video stream.")
        return

    logger.info("Camera opened successfully. Starting frame publication...")
    
    # Set a target frame rate (e.g., 30 FPS)
    FPS = 30
    FRAME_INTERVAL = 1.0 / FPS
    
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                logger.warning("Failed to grab frame, breaking loop.")
                break
            
            # Get frame dimensions
            height, width, channels = frame.shape
            
            # Create a Protobuf message
            frame_data = frame_data_pb2.FrameData()
            frame_data.height = height
            frame_data.width = width
            frame_data.channels = channels
            frame_data.frame = frame.tobytes() # Convert numpy array to raw bytes

            # Serialize the Protobuf message to a byte string
            msg = frame_data.SerializeToString()
            
            # Publish the message with its topic
            socket.send_multipart([zmq_topic, msg])
            logger.info(f"Published frame with size: {len(msg)} bytes")
            
            # Wait to maintain the target frame rate
            time.sleep(FRAME_INTERVAL)

    except KeyboardInterrupt:
        logger.info("Interrupted by user. Shutting down.")
    finally:
        # Clean up resources
        logger.info("Cleaning up resources...")
        cap.release()
        socket.close()
        context.term()
        logger.info("Publisher shut down successfully.")

if __name__ == "__main__":
    main()