# aruco_estimator_node.py (Versi Kalibrasi YAML)
import cv2
from cv2.aruco import ArucoDetector, getPredefinedDictionary, DetectorParameters
import numpy as np
import zmq
import yaml
import threading
import time
import logging
from managednode import ManagedNode
from shared_enums import NodeState

class ArucoEstimatorNode(ManagedNode):
    # (Metode __init__ tidak berubah)
    def __init__(self, node_name="aruco_estimator", config_path="aruco_estimator.yaml"):
        super().__init__(node_name)
        self.config_path = config_path
        self.config = None
        self.camera_sub = None
        self.distance_pub = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detector = None
        self.frame_shape = None
        self.dtype = np.uint8
        self.processing_thread = None
        self.active_event = threading.Event()
        self.display_frame = None
        self.frame_lock = threading.Lock()

    def on_configure(self) -> bool:
        self.logger.info("Configuring...")
        try:
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f)

            zmq_config = self.config['zmq']
            detection_config = self.config['detection']

            # (Bagian ZMQ tidak berubah)
            self.camera_sub = self.context.socket(zmq.SUB)
            self.camera_sub.connect(zmq_config['sub_frame_url'])
            self.camera_sub.setsockopt_string(zmq.SUBSCRIBE, zmq_config['sub_frame_topic'])
            self.distance_pub = self.context.socket(zmq.PUB)
            self.distance_pub.bind(zmq_config['pub_distance_url'])
            
            # --- BAGIAN YANG DIUBAH: Logika Pemuatan Kalibrasi ---
            calibration_filepath = detection_config['calibration_file']
            self.logger.info(f"Mencoba memuat file kalibrasi YAML: {calibration_filepath}")
            
            fs = cv2.FileStorage(calibration_filepath, cv2.FILE_STORAGE_READ)
            if not fs.isOpened():
                raise IOError(f"Gagal membuka file kalibrasi: {calibration_filepath}")

            self.camera_matrix = fs.getNode("camera_matrix").mat()
            self.dist_coeffs = fs.getNode("dist_coeff").mat()
            fs.release()
            self.logger.info("File kalibrasi .yaml berhasil dimuat.")
            # --------------------------------------------------------
            
            # (Sisa dari on_configure tidak berubah)
            aruco_dict = getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            parameters = DetectorParameters()
            parameters.adaptiveThreshConstant = 7
            self.detector = ArucoDetector(aruco_dict, parameters)
            
            self.frame_shape = (detection_config['frame_height'], detection_config['frame_width'], 3)
            return True
        except Exception as e:
            self.logger.error(f"Configuration failed: {e}")
            return False

    # (Sisa dari kelas dan program tidak ada yang berubah, gunakan versi lengkap dari respons sebelumnya)
    def on_activate(self) -> bool:
        self.logger.info("Activating processing loop...")
        self.active_event.set()
        self.processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.processing_thread.start()

        # --- PRINT TAMBAHAN SAAT AKTIF ---
        self.logger.info(">>> Node Aruco Estimator Telah AKTIF <<<")

        return True

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating processing loop...")
        self.active_event.clear()
        if self.processing_thread: self.processing_thread.join(timeout=1.0)

        # --- PRINT TAMBAHAN SAAT NONAKTIF ---
        self.logger.info(">>> Node Aruco Estimator Telah DI-NONAKTIFKAN <<<")

        return True

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down internal resources...")
        if self.state == NodeState.ACTIVE: self.on_deactivate()
        if self.camera_sub: self.camera_sub.close()
        if self.distance_pub: self.distance_pub.close()
        return True

    def _processing_loop(self):
        self.logger.info("Processing loop started.")
        local_poller = zmq.Poller()
        local_poller.register(self.camera_sub, zmq.POLLIN)
        
        detection_config = self.config['detection']
        zmq_config = self.config['zmq']
        
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
                            corners, detection_config['known_marker_width_m'], self.camera_matrix, self.dist_coeffs
                        )
                        direct_dist = tvecs[0][0][2]
                        if direct_dist > detection_config['camera_height_m']:
                            ground_distance = np.sqrt(direct_dist**2 - detection_config['camera_height_m']**2)
                            self.distance_pub.send_string(zmq_config['pub_distance_topic'], flags=zmq.SNDMORE)
                            self.distance_pub.send_string(f"{ground_distance}")

                            # cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                            # cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], 0.1)
                            # cv2.putText(frame, f"Jarak: {ground_distance:.2f} m", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                except zmq.Again:
                    continue

                # with self.frame_lock:
                #     self.display_frame = frame.copy()
        
        local_poller.unregister(self.camera_sub)
        self.logger.info("Processing loop stopped.")

if __name__ == "__main__":
    node = ArucoEstimatorNode()
    node.run()
    # node_thread = threading.Thread(target=node.run, daemon=True)
    # node_thread.start()

    # print("Memulai Tampilan Visual. Tekan 'q' pada jendela untuk keluar.")
    # while node_thread.is_alive():
    #     frame_to_show = None
    #     with node.frame_lock:
    #         if node.display_frame is not None:
    #             frame_to_show = node.display_frame.copy()

    #     if frame_to_show is not None:
    #         cv2.imshow("Aruco Estimator - Visual Feed", frame_to_show)

    #     key = cv2.waitKey(1) & 0xFF
    #     if key == ord('q'):
    #         print("Tombol 'q' ditekan, mengirim sinyal shutdown...")
    #         node.shutdown_event.set()
    #         break
    
    # node_thread.join(timeout=2.0)
    # cv2.destroyAllWindows()
    # print("Tampilan visual ditutup.")