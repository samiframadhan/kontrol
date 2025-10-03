import cv2
import zmq
import logging
import threading
import numpy as np
import ast
import time

class ZmqFrameSubscriber:
    """
    Threaded ZMQ SUB receiver that reconstructs frames from multipart messages
    (topic, timestamp, shape, dtype, image_bytes) and keeps the latest one.
    Non-blocking getter returns the most recent frame.
    """
    def __init__(self, context: zmq.Context, url: str, topic: str, name: str = "frame_sub"):
        self._context = context
        self._url = url
        self._topic = topic
        self._name = name
        self._socket = None
        self._thread = None
        self._shutdown = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._timestamp = None
        self._latency_ms = None
        self._topic = None
        self._logger = logging.getLogger(f"ZmqFrameSubscriber[{self._name}]")

    def start(self):
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)
        self._socket.connect(self._url)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, self._topic)
        self._logger.info(f"Connected: url={self._url}, topic='{self._topic}'")

        self._thread = threading.Thread(target=self._loop, name=f"{self._name}_Thread", daemon=True)
        self._thread.start()

    def get_timestamp(self):
        with self._lock:
            return self._timestamp
        
    def get_latency_ms(self):
        with self._lock:
            return self._latency_ms
        
    def _loop(self):
        """
        Receives multipart messages and reconstructs the NumPy array for the frame.
        """
        while not self._shutdown.is_set():
            try:
                (
                    topic,
                    timestamp_bytes,
                    shape_bytes,
                    dtype_bytes,
                    image_bytes,
                ) = self._socket.recv_multipart()
            except zmq.Again:
                continue
            except Exception as e:
                self._logger.warning(f"recv error: {e}")
                continue

            try:
                # 1. Evaluate the shape string into a tuple
                shape = ast.literal_eval(shape_bytes.decode('utf-8'))
                # 2. Get the data type
                dtype = np.dtype(dtype_bytes.decode('utf-8'))
                # 3. Create the NumPy array from the raw bytes, dtype, and shape
                frame = np.frombuffer(image_bytes, dtype=dtype).reshape(shape)

                with self._lock:
                    self._latest = frame
                    self._timestamp = timestamp_bytes
                    self._latency_ms = (time.time_ns() - int(timestamp_bytes.decode('utf-8'))) / 1_000_000
                    self._topic = topic
            except Exception as e:
                # <<< MODIFIED: Updated log message for clarity
                self._logger.warning(f"Failed to decode/reshape frame: {e}")

    def get_latest(self):
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def stop(self):
        self._shutdown.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
        self._logger.info("Stopped.")