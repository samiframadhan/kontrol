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
from config_mixin import ConfigMixin
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
        self.rs_ctx = rs.context()
        
    def start(self):
        logging.info("Starting RealSense camera...")
        devices = self.rs_ctx.query_devices()
        device_found = False
        for dev in devices:
            if self.config.get('device_name') in dev.get_info(rs.camera_info.name):
                self.rs_config.enable_device(dev.get_info(rs.camera_info.serial_number))
                device_found = True
                break
        if not device_found:
            raise RuntimeError(f"RealSense device named '{self.config.get('device_name')}' not found.")
        
        self.rs_config.enable_stream(rs.stream.color, self.config['frame_width'], self.config['frame_height'], rs.format.bgr8, self.config['frame_fps'])
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
        self.config = cam_config
        self.source = source
        self.cap = None
    def start(self):
        logging.info(f"Starting OpenCV camera with source: {self.source}...")
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            raise IOError(f"Cannot open OpenCV source: {self.source}")
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
# camera_node.py (Updated sections only)
from config_mixin import ConfigMixin

class CameraNode(ManagedNode, ConfigMixin):
    def __init__(self, node_name: str, config_path: str = "config.yaml", camera_type: str = "forward"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        self.camera_type = camera_type
        self.pub_socket = None
        self.camera = None
        self.publishing_thread = None
        self.publishing_active = threading.Event()

    def on_configure(self) -> bool:
        self.logger.info(f"Configuring Camera Node ({self.camera_type})...")
        try:
            # Setup ZMQ Publisher
            url_key = 'camera_frame_url' if self.camera_type == 'forward' else 'camera_frame_reverse_url'
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(self.get_zmq_url(url_key))
            self.logger.info(f"ZMQ Publisher bound to {self.get_zmq_url(url_key)}")
            
            return True
        except Exception as e:
            self.logger.error(f"Error during configuration: {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating Camera Node...")
        try:
            camera_config = self.get_camera_config(self.camera_type)
            source = camera_config.get('source', 0)
            camera_source_type = camera_config.get('camera_source')

            if camera_source_type == 'realsense':
                self.camera = RealSenseCamera(camera_config)
            elif camera_source_type == 'opencv':
                self.camera = OpenCVCamera(camera_config, source)
            else:
                self.logger.error(f"Unsupported camera source: '{camera_source_type}'")
                return False

            self.camera.start()
            self.logger.info(f"Camera '{camera_source_type}' started successfully.")

            self.publishing_active.set()
            self.publishing_thread = threading.Thread(target=self._publish_loop, daemon=True)
            self.publishing_thread.start()
            
            self.logger.info("Frame publishing thread started.")
            return True
        except Exception as e:
            self.logger.error(f"Failed to activate Camera Node: {e}")
            self.camera = None
            return False

    def _publish_loop(self):
        camera_config = self.get_camera_config(self.camera_type)
        
        frame_width = camera_config.get('frame_width', 640)
        frame_height = camera_config.get('frame_height', 480)
        fps = camera_config.get('frame_fps', 30)
        
        topic_key = 'camera_frame_topic' if self.camera_type == 'forward' else 'camera_frame_reverse_topic'
        frame_topic = self.get_zmq_topic(topic_key)

        self.logger.info(f"Publishing topic '{frame_topic}' at approximately {fps} FPS.")
        
        while self.publishing_active.is_set():
            if self.camera is None:
                self.logger.error("Camera is not initialized. Stopping publish loop.")
                break

            frame = self.camera.get_frame()
            if frame is None:
                self.logger.warning("Failed to get frame from camera.")
                break

            if frame.shape[1] != frame_width or frame.shape[0] != frame_height:
                frame = cv2.resize(frame, (frame_width, frame_height))

            try:
                self.pub_socket.send_string(frame_topic, flags=zmq.SNDMORE)
                self.pub_socket.send(frame.tobytes())
            except zmq.ZMQError as e:
                self.logger.error(f"ZMQ error while sending frame: {e}")
                break
        
        self.logger.info("Publish loop has terminated.")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-type', type=str, default='forward', 
                       choices=['forward', 'reverse'], help='Camera type to use')
    args = parser.parse_args()

    node_name = f"camera_node_{args.camera_type}"
    camera_node = CameraNode(node_name=node_name, camera_type=args.camera_type)
    camera_node.run()

if __name__ == "__main__":
    main()