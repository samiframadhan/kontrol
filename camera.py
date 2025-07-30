# camera.py
import cv2
import zmq
import yaml
import time
import logging
import threading
from typing import Dict, Any
import numpy as np
import pyrealsense2 as rs
from abc import ABC, abstractmethod

# Assuming managednode.py is in the same directory or accessible in the python path
from managednode import ManagedNode

# -------------------------------------------------------------------
# --- 2. CORE CLASSES -----------------------------------------------
# -------------------------------------------------------------------

class Camera(ABC):
    @abstractmethod
    def start(self): pass
    @abstractmethod
    def get_frame(self): pass
    @abstractmethod
    def stop(self): pass

class RealSenseCamera(Camera):
    """Camera implementation for Intel RealSense."""
    def __init__(self, cam_config: dict):
        self.config = cam_config
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.rs_config.enable_stream(rs.stream.color, self.config['frame_width'], self.config['frame_height'], rs.format.bgr8, self.config['frame_fps'])
        
    def start(self):
        logging.info("Starting RealSense camera...")
        
        self.pipeline.start(self.rs_config)
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            logging.warning("No color frame received from RealSense camera.")
            return None
        return np.asanyarray(color_frame.get_data())
    def stop(self):
        logging.info("Stopping RealSense camera.")
        self.pipeline.stop()

class OpenCVCamera(Camera):
    """Camera implementation for any source supported by OpenCV."""
    def __init__(self, cam_config: dict, source):
        self.config = cam_config; self.source = source; self.cap = None
    def start(self):
        logging.info(f"Starting OpenCV camera with source: {self.source}...")
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened(): raise IOError(f"Cannot open OpenCV source: {self.source}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['frame_width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['frame_height'])
        self.cap.set(cv2.CAP_PROP_FPS, self.config['frame_fps'])
    def get_frame(self):
        ret, frame = self.cap.read()
        return frame if ret else None
    def stop(self):
        if self.cap:
            logging.info("Stopping OpenCV camera.")
            self.cap.release()

class ZMQCamera(Camera):
    """Camera implementation for a ZMQ subscription source."""
    def __init__(self, cam_config: dict, zmq_sub_config: dict, context: zmq.Context):
        self.config = cam_config
        self.zmq_config = zmq_sub_config
        self.context = context # Use a shared ZMQ context
        self.socket = None
        self.poller = None
        self.frame_shape = (self.config['frame_height'], self.config['frame_width'], 3)
        self.dtype = np.uint8

    def start(self):
        logging.info(f"Connecting to ZMQ frame publisher at {self.zmq_config['sub_frame_url']}...")
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.zmq_config['sub_frame_url'])
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self.zmq_config['sub_frame_topic'])
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        logging.info(f"Subscribed to frame topic: '{self.zmq_config['sub_frame_topic']}'")

    def get_frame(self):
        try:
            # Poll with a timeout to avoid blocking indefinitely
            socks = dict(self.poller.poll(100))
            if self.socket in socks and socks[self.socket] == zmq.POLLIN:
                topic = self.socket.recv_string()
                frame_bytes = self.socket.recv()
                frame = np.frombuffer(frame_bytes, dtype=self.dtype)
                return frame.reshape(self.frame_shape)
        except Exception as e:
            logging.error(f"Error processing ZMQ frame: {e}")
        return None

    def stop(self):
        if self.socket:
            logging.info("Closing ZMQ frame subscriber socket.")
            self.socket.close()


class CameraNode(ManagedNode):
    """
    A managed node for capturing frames from a camera or video file
    and publishing them over a ZMQ PUB socket.
    """
    def __init__(self, node_name: str, config_path: str = 'camera.yaml'):
        super().__init__(node_name)
        self.config_path = config_path
        self.config = None
        self.pub_socket = None
        self.cap = None
        self.publishing_thread = None
        self.publishing_active = threading.Event()

    def on_configure(self) -> bool:
        """
        Loads configuration and sets up ZMQ resources.
        """
        self.logger.info("Configuring Camera Node...")
        try:
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            
            # --- ZMQ Publisher Setup ---
            zmq_config = self.config.get('zmq', {})
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(zmq_config['pub_frame_url'])
            self.logger.info(f"ZMQ Publisher bound to {zmq_config['pub_frame_url']}")
            
            return True
        except FileNotFoundError:
            self.logger.error(f"Configuration file not found at: {self.config_path}")
            return False
        except Exception as e:
            self.logger.error(f"Error during configuration: {e}")
            return False

    def on_activate(self) -> bool:
        """
        Opens the camera and starts the frame publishing thread.
        """
        self.logger.info("Activating Camera Node...")
        camera_config = self.config.get('camera', {})
        source = camera_config.get('source', 0)
        
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            self.logger.error(f"Cannot open video source: {source}")
            return False

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_config.get('frame_width', 640))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_config.get('frame_height', 480))
        self.cap.set(cv2.CAP_PROP_FPS, camera_config.get('frame_fps', 30))

        self.logger.info(f"Camera source '{source}' opened successfully.")
        
        # Start the publishing loop in a separate thread
        self.publishing_active.set()
        self.publishing_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.publishing_thread.start()
        
        self.logger.info("Frame publishing thread started.")
        return True

    def on_deactivate(self) -> bool:
        """
        Stops the publishing thread and releases the camera.
        """
        self.logger.info("Deactivating Camera Node...")
        self.publishing_active.clear() # Signal the loop to stop
        if self.publishing_thread and self.publishing_thread.is_alive():
            self.publishing_thread.join(timeout=1.0)
        
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.logger.info("Camera released.")
        
        self.cap = None
        self.publishing_thread = None
        return True

    def on_shutdown(self) -> bool:
        """
        Cleans up all resources.
        """
        self.logger.info("Shutting down Camera Node...")
        if self.state == "active":
            self.on_deactivate() # First, deactivate to stop threads and release camera

        if self.pub_socket:
            self.pub_socket.close()
            self.logger.info("Publisher socket closed.")
            
        return True

    def _publish_loop(self):
        """
        The main loop for capturing and publishing frames.
        """
        camera_config = self.config.get('camera', {})
        zmq_config = self.config.get('zmq', {})
        
        frame_width = camera_config.get('frame_width', 640)
        frame_height = camera_config.get('frame_height', 480)
        fps = camera_config.get('frame_fps', 30)
        sleep_interval = 1.0 / fps
        frame_topic = zmq_config.get('frame_topic', 'frame')

        self.logger.info(f"Publishing topic '{frame_topic}' at approximately {fps} FPS.")
        
        while self.publishing_active.is_set():
            if not self.cap or not self.cap.isOpened():
                self.logger.warning("Camera is not available. Stopping publish loop.")
                break

            ret, frame = self.cap.read()
            if not ret:
                self.logger.warning("End of video file or camera disconnected. Stopping.")
                break

            # Resize if necessary (fallback)
            if frame.shape[1] != frame_width or frame.shape[0] != frame_height:
                 frame = cv2.resize(frame, (frame_width, frame_height))

            # Publish the frame
            try:
                self.pub_socket.send_string(frame_topic, flags=zmq.SNDMORE)
                self.pub_socket.send(frame.tobytes())
            except zmq.ZMQError as e:
                self.logger.error(f"ZMQ error while sending frame: {e}")
                break

            time.sleep(sleep_interval)
        
        self.logger.info("Publish loop has terminated.")


def main():
    """
    Main function to instantiate and run the CameraNode.
    """
    # Note: Logging is configured within the ManagedNode base class now.
    camera_node = CameraNode(node_name="camera_node", config_path="camera.yaml")
    camera_node.run()

if __name__ == "__main__":
    main()