# frame_viewer.py
import cv2
import zmq
import numpy as np
import logging
import yaml  # <-- NEW: Import YAML library
import frame_data_pb2  # Assumes frame_data.proto is compiled

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

def main():
    """
    Connects to a ZMQ publisher by reading settings from config.yaml 
    and displays the received video frames.
    """
    setup_logging()
    logger = logging.getLogger("FrameViewer")
    
    # --- Configuration ---
    config = load_config()
    if not config:
        return

    # IMPORTANT: Use the IP address of the machine running the linefollowing_node.
    # This will replace the '*' in the ZMQ URL from the config file.
    PUBLISHER_IP = "192.168.55.1" 
    
    try:
        # Get URL and topic from the loaded config
        zmq_config = config.get('zmq', {})
        zmq_url_template = zmq_config.get('frame_publish_url')
        zmq_topic = zmq_config.get('frame_publish_topic')

        if not zmq_url_template or not zmq_topic:
            logger.error("'frame_publish_url' or 'frame_publish_topic' not found in config.yaml")
            return

        # Replace the wildcard '*' with the actual publisher IP
        zmq_url = zmq_url_template.replace('*', PUBLISHER_IP)

    except KeyError as e:
        logger.error(f"Missing required configuration key: {e}")
        return

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    
    # Connect to the publisher
    logger.info(f"Connecting to publisher at {zmq_url}")
    socket.connect(zmq_url)
    
    # Subscribe to the topic
    socket.setsockopt_string(zmq.SUBSCRIBE, zmq_topic)
    
    logger.info(f"Viewer started. Subscribed to topic '{zmq_topic}'. Waiting for frames...")
    
    try:
        while True:
            # Receive the message parts (topic and the protobuf message)
            topic, msg = socket.recv_multipart()
            
            # Deserialize the protobuf message
            frame_data = frame_data_pb2.FrameData()
            frame_data.ParseFromString(msg)
            
            # Reconstruct the numpy array from the raw bytes
            frame = np.frombuffer(frame_data.frame, dtype=np.uint8).reshape(
                (frame_data.height, frame_data.width, frame_data.channels)
            )
            
            # Display the frame
            cv2.imshow("Remote Frame Viewer", frame)
            
            # Check for 'q' key press to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logger.info("'q' pressed, shutting down viewer.")
                break
                
    except KeyboardInterrupt:
        logger.info("Interrupted by user. Shutting down.")
    finally:
        # Clean up resources
        cv2.destroyAllWindows()
        socket.close()
        context.term()
        logger.info("Viewer shut down successfully.")

if __name__ == "__main__":
    main()