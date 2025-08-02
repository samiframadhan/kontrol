# aruco_estimator_node.py
import cv2
from cv2.aruco import ArucoDetector, getPredefinedDictionary, DetectorParameters
import numpy as np
import zmq
import threading
import time
import logging
from managednode import ManagedNode
from shared_enums import NodeState
from config_mixin import ConfigMixin

class ArucoEstimatorNode(ManagedNode, ConfigMixin):
    def __init__(self, node_name="aruco_estimator", config_path="config.yaml"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        self.camera_sub = None
        self.distance_pub = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detector = None
        self.frame_shape = None
        self.dtype = np.uint8
        self.processing_thread = None
        self.active_event = threading.Event()

    def on_configure(self) -> bool:
        self.logger.info("Configuring ArUco Estimator Node...")
        try:
            # Get ArUco-specific configuration
            aruco_config = self.get_section_config('aruco')
            
            # Setup ZMQ connections
            self.camera_sub = self.context.socket(zmq.SUB)
            self.camera_sub.connect(self.get_zmq_url('camera_frame_url'))
            self.camera_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('camera_frame_topic'))
            
            self.distance_pub = self.context.socket(zmq.PUB)
            self.distance_pub.bind(self.get_zmq_url('distance_url'))
            
            # Load calibration file
            calibration_filepath = aruco_config['calibration_file']
            self.logger.info(f"Loading calibration file: {calibration_filepath}")
            
            fs = cv2.FileStorage(calibration_filepath, cv2.FILE_STORAGE_READ)
            if not fs.isOpened():
                raise IOError(f"Failed to open calibration file: {calibration_filepath}")

            self.camera_matrix = fs.getNode("camera_matrix").mat()
            self.dist_coeffs = fs.getNode("dist_coeff").mat()
            fs.release()
            self.logger.info("Calibration file loaded successfully.")
            
            # Setup ArUco detector
            aruco_dict = getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            parameters = DetectorParameters()
            parameters.adaptiveThreshConstant = 7
            self.detector = ArucoDetector(aruco_dict, parameters)
            
            self.frame_shape = (aruco_config['frame_height'], aruco_config['frame_width'], 3)
            return True
            
        except Exception as e:
            self.logger.error(f"Configuration failed: {e}")
            return False

    def on_activate(self) -> bool:
        try:
            self.logger.info("Activating ArUco Estimator Node...")
            self.active_event.set()
            self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
            self.processing_thread.start()
            self.logger.info("ArUco Estimator Node activated.")
        except Exception as e:
            self.logger.error(f"Activation failed: {e}")
            return False
        return True

    def on_deactivate(self) -> bool:
        try:
            self.logger.info("Deactivating ArUco Estimator Node...")
            self.active_event.clear()
            if self.processing_thread:
                self.processing_thread.join(timeout=1.0)
            self.logger.info("ArUco Estimator Node deactivated.")
        except Exception as e:
            self.logger.error(f"Deactivation failed: {e}")
            return False
        return True

    def on_shutdown(self) -> bool:
        try:
            self.logger.info("Shutting down ArUco Estimator Node...")
            if self.state == NodeState.ACTIVE:
                self.on_deactivate()
            if self.camera_sub:
                self.camera_sub.close()
            if self.distance_pub:
            self.distance_pub.close()
        except Exception as e:
            self.logger.error(f"Shutdown failed: {e}")
            return False
        return True

    def _processing_loop(self):
        self.logger.info("ArUco processing loop started.")
        local_poller = zmq.Poller()
        local_poller.register(self.camera_sub, zmq.POLLIN)
        
        aruco_config = self.get_section_config('aruco')
        
        while self.active_event.is_set():
            socks = dict(local_poller.poll(100))
            if self.camera_sub in socks:
                try:
                    topic = self.camera_sub.recv_string(flags=zmq.NOBLOCK)
                    frame_bytes = self.camera_sub.recv(flags=zmq.NOBLOCK)
                    frame = np.frombuffer(frame_bytes, dtype=self.dtype).reshape(self.frame_shape).copy()
                    
                    corners, ids, _ = self.detector.detectMarkers(frame)
                    if ids is not None:
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                            corners, aruco_config['known_marker_width_m'], 
                            self.camera_matrix, self.dist_coeffs
                        )
                        direct_dist = tvecs[0][0][2]
                        if direct_dist > aruco_config['camera_height_m']:
                            ground_distance = np.sqrt(direct_dist**2 - aruco_config['camera_height_m']**2)
                            self.distance_pub.send_string(self.get_zmq_topic('distance_topic'), flags=zmq.SNDMORE)
                            self.distance_pub.send_string(f"{ground_distance}")

                except zmq.Again:
                    continue
        
        local_poller.unregister(self.camera_sub)
        self.logger.info("ArUco processing loop stopped.")

if __name__ == "__main__":
    node = ArucoEstimatorNode()
    node.run()