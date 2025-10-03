# image_viewer.py
import cv2
import zmq
import logging
import yaml
import threading
import queue
import simplejpeg as sjpg
import time # <<< ADDED

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
                # <<< MODIFIED: Receive three message parts now
                topic, timestamp_bytes, msg_bytes = socket.recv_multipart()
                
                try:
                    # <<< MODIFIED: Put a tuple (timestamp, jpeg_bytes) into the queue
                    message_queue.put((timestamp_bytes, msg_bytes), block=False)
                except queue.Full:
                    try:
                        message_queue.get_nowait()
                        message_queue.put((timestamp_bytes, msg_bytes), block=False)
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

def main():
    """
    Displays video frames from a queue and overlays the calculated latency.
    """
    setup_logging()
    logger = logging.getLogger("FrameViewer")
    
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
                # <<< MODIFIED: Get the tuple from the queue
                timestamp_bytes, jpeg_bytes = message_queue.get(timeout=1)
                
                # <<< ADDED: Calculate latency
                recv_time = time.time_ns()
                send_time = int(timestamp_bytes.decode('utf-8'))
                latency_ms = (recv_time - send_time) / 1_000_000_000
                
                frame = sjpg.decode_jpeg(jpeg_bytes, colorspace='BGR')
                
                # <<< ADDED: Draw latency on the frame
                latency_text = f"Latency: {latency_ms:.1f} ms"
                cv2.putText(frame, latency_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
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