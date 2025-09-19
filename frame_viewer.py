# frame_viewer.py
import cv2
import zmq
import numpy as np
import logging
import yaml
import frame_data_pb2  # Assumes frame_data.proto is compiled
import threading
import queue

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

# --- MODIFIED: ZMQ Subscriber Thread Function ---
def zmq_subscriber_thread(zmq_url, zmq_topic, message_queue, shutdown_event):
    """
    This function runs in a separate thread. It connects to the ZMQ publisher,
    receives raw messages, and puts them into a thread-safe queue.
    It does NOT do any deserialization.
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
                # Receive the message parts (topic and the raw protobuf message)
                topic, msg = socket.recv_multipart()
                
                # --- CHANGED: Put the raw message directly into the queue ---
                try:
                    message_queue.put(msg, block=False)
                except queue.Full:
                    logger.warning("Message queue is full, dropping a message.")
                    continue

            except zmq.Again:
                continue
    
    finally:
        logger.info("Subscriber thread shutting down.")
        socket.close()
        context.term()

def main():
    """
    Sets up a ZMQ subscriber in a separate thread and displays
    the received video frames from a queue in the main thread.
    """
    setup_logging()
    logger = logging.getLogger("FrameViewer")
    
    config = load_config()
    if not config:
        return

    PUBLISHER_IP = "127.0.0.1"
    
    try:
        zmq_config = config.get('zmq', {})
        zmq_url_template = zmq_config.get('camera_frame_url')
        zmq_topic = zmq_config.get('camera_frame_topic')

        if not zmq_url_template or not zmq_topic:
            logger.error("'camera_frame_url' or 'camera_frame_topic' not found in config.yaml")
            return

        zmq_url = zmq_url_template.replace('*', PUBLISHER_IP)

    except KeyError as e:
        logger.error(f"Missing required configuration key: {e}")
        return

    # --- CHANGED: Renamed queue for clarity ---
    # This queue will hold raw protobuf messages (bytes)
    message_queue = queue.Queue(maxsize=5) 
    
    shutdown_event = threading.Event()

    subscriber = threading.Thread(
        target=zmq_subscriber_thread,
        args=(zmq_url, zmq_topic, message_queue, shutdown_event), # <-- Pass message_queue
        daemon=True
    )
    subscriber.start()
    
    logger.info("Main thread started. Waiting for messages from subscriber...")
    
    try:
        while True:
            try:
                # Get a raw message from the queue.
                msg = message_queue.get(timeout=1)
                
                frame_data = frame_data_pb2.FrameData()
                frame_data.ParseFromString(msg)
                
                # Reconstruct the numpy array from the raw bytes
                frame = np.frombuffer(frame_data.frame, dtype=np.uint8).reshape(
                    (frame_data.height, frame_data.width, frame_data.channels)
                )
                # --- End of moved logic ---

                cv2.imshow("Remote Frame Viewer", frame)
                
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
        
        cv2.destroyAllWindows()
        logger.info("Viewer shut down successfully.")

if __name__ == "__main__":
    main()