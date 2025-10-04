# camera.py

import cv2
import zmq
import logging
import multiprocessing as mp
import numpy as np
import time
import simplejpeg as sjpg

class ZMQFrameSubscriber:
    """
    Manages a separate process for ZMQ frame subscription and JPEG decoding.

    This class starts a dedicated process to handle ZMQ communication,
    preventing the main application from being blocked by network I/O.
    It uses a multiprocessing.Queue to pass frames efficiently from the
    subscriber process to the main process.
    """
    def __init__(self, url: str, topic: str, name: str = "jpeg_sub"):
        self._url = url
        self._topic = topic
        self._name = name
        self._logger = logging.getLogger(f"ZmqJpegSubscriber[{self._name}]")

        # Process and communication primitives
        self._process = None
        self._queue = None  # type: mp.Queue
        self._shutdown_event = None  # type: mp.Event

        # Thread-safe attributes for the latest frame data in the main process
        self._lock = mp.Lock()
        self._latest_frame = None
        self._timestamp_ns = None
        self._latency_ms = None

    @staticmethod
    def _subscriber_loop(url: str, topic: str, queue: mp.Queue, shutdown_event: mp.Event):
        """
        This function runs in a separate process.
        It connects to the ZMQ publisher, receives messages, and puts them in the queue.
        """
        # Each process needs its own ZMQ context and logger configuration
        proc_logger = logging.getLogger(f"ZmqSubscriberProcess[{topic}]")
        logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1-second timeout for recv
        socket.connect(url)
        socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        proc_logger.info(f"Subscriber process connected: url={url}, topic='{topic}'")

        try:
            while not shutdown_event.is_set():
                try:
                    # Receive multipart message (topic, timestamp, jpeg_bytes)
                    _, timestamp_bytes, jpeg_bytes = socket.recv_multipart()

                    # Put the raw data into the shared queue for the main process
                    # This is a non-blocking operation to prevent the subscriber from stalling
                    # if the main process is slow to consume frames.
                    try:
                        # Clear the queue to only keep the latest frame
                        while not queue.empty():
                            queue.get_nowait()
                        queue.put_nowait((timestamp_bytes, jpeg_bytes))
                    except mp.queues.Full:
                        # This should rarely happen with the clearing logic, but is safe to ignore.
                        pass

                except zmq.Again:
                    # Expected exception on timeout, just continue waiting
                    continue
                except Exception as e:
                    proc_logger.warning(f"Subscriber recv error: {e}")

        finally:
            proc_logger.info("Subscriber process shutting down.")
            socket.close()
            context.term()

    def start(self):
        """Starts the ZMQ subscriber process."""
        if self._process is not None and self._process.is_alive():
            self._logger.warning("Process is already running.")
            return

        self._queue = mp.Queue(maxsize=1)
        self._shutdown_event = mp.Event()
        
        self._process = mp.Process(
            target=self._subscriber_loop,
            args=(self._url, self._topic, self._queue, self._shutdown_event),
            name=f"{self._name}_Process",
            daemon=True
        )
        self._process.start()
        self._logger.info("Subscriber process started.")

    def _update_latest_frame(self):
        """
        Internal method to get the latest item from the queue and decode it.
        This is called by the public getter methods.
        """
        try:
            timestamp_bytes, jpeg_bytes = self._queue.get_nowait()
            
            # Decode the JPEG bytes into a BGR NumPy array
            frame = sjpg.decode_jpeg(jpeg_bytes, colorspace='BGR')
            
            timestamp_ns = int(timestamp_bytes.decode('utf-8'))
            latency_ms = (time.time_ns() - timestamp_ns) / 1_000_000

            # Lock before updating the shared state
            with self._lock:
                self._latest_frame = frame
                self._timestamp_ns = timestamp_ns
                self._latency_ms = latency_ms

        except mp.queues.Empty:
            # No new frame is available, do nothing
            pass
        except Exception as e:
            self._logger.warning(f"Failed to get or decode frame from queue: {e}")


    def get_latest(self) -> np.ndarray | None:
        """
        Non-blocking call to get the most recent frame.
        Returns the latest available frame or None if no frame has been received.
        """
        self._update_latest_frame()
        with self._lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def get_timestamp_ns(self) -> int | None:
        """Returns the nanosecond timestamp of the latest frame."""
        self._update_latest_frame()
        with self._lock:
            return self._timestamp_ns
        
    def get_latency_ms(self) -> float | None:
        """Returns the calculated latency in milliseconds of the latest frame."""
        self._update_latest_frame()
        with self._lock:
            return self._latency_ms

    def stop(self):
        """Signals the subscriber process to shut down and waits for it to terminate."""
        self._logger.info("Stopping subscriber process...")
        if self._shutdown_event:
            self._shutdown_event.set()
        
        if self._process:
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                self._logger.warning("Process did not exit gracefully. Terminating.")
                self._process.terminate()
                self._process.join()

        self._process = None
        self._queue = None
        self._logger.info("Stopped.")