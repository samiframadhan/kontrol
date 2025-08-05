# linefollowing_node.py (Original 518 lines + Aruco Integration)
import cv2
from cv2 import aruco
import datetime
import math
import numpy as np
import zmq
import time
import logging
import threading
import json
import pyrealsense2 as rs 
from dataclasses import dataclass
from abc import ABC, abstractmethod

# Import the protobuf definitions
import steering_command_pb2
import frame_data_pb2

from managednode import ManagedNode
from config_mixin import ConfigMixin

def setup_logging():
    """Configures the logging for the application."""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# -------------------------------------------------------------------
# --- CLASS FOR MANAGING MULTIPLE REALSENSE CAMERAS (Original) ------
# -------------------------------------------------------------------
class MultiRealSenseCamera:
    """Manages multiple Intel RealSense cameras (e.g., D435 for forward, D415 for reverse)."""
    def __init__(self, cam_config: dict):
        self.config = cam_config
        self.rs_ctx = rs.context()
        self.pipelines = {}  # Using a dict to map 'forward'/'reverse' to pipelines
        self.serial_numbers = {}  # To store found serial numbers

        # Device identification from config
        self.forward_cam_name = self.config.get('forward_camera_name', 'D435')
        self.reverse_cam_name = self.config.get('reverse_camera_name', 'D415')
        logging.info(f"Seeking Forward Cam: '{self.forward_cam_name}', Reverse Cam: '{self.reverse_cam_name}'")

    def start(self):
        logging.info("Starting RealSense cameras...")
        devices = self.rs_ctx.query_devices()
        
        found_devs = {'forward': False, 'reverse': False}

        for dev in devices:
            dev_name = dev.get_info(rs.camera_info.name)
            serial = dev.get_info(rs.camera_info.serial_number)
            
            cam_type = None
            if self.forward_cam_name in dev_name and not found_devs['forward']:
                cam_type = 'forward'
            elif self.reverse_cam_name in dev_name and not found_devs['reverse']:
                cam_type = 'reverse'

            if cam_type:
                logging.info(f"Found {cam_type} camera: {dev_name} (Serial: {serial})")
                pipeline = rs.pipeline(self.rs_ctx)
                cfg = rs.config()
                cfg.enable_device(serial)
                cfg.enable_stream(rs.stream.color, 
                                  self.config['frame_width'], 
                                  self.config['frame_height'], 
                                  rs.format.bgr8, 
                                  self.config['frame_fps'])
                pipeline.start(cfg)
                self.pipelines[cam_type] = pipeline
                self.serial_numbers[cam_type] = serial
                found_devs[cam_type] = True

        if not found_devs['forward']:
            logging.error(f"Forward RealSense device containing name '{self.forward_cam_name}' not found.")
        if not found_devs['reverse']:
            logging.error(f"Reverse RealSense device containing name '{self.reverse_cam_name}' not found.")
        
        if not all(found_devs.values()):
             raise RuntimeError("Could not find all required RealSense devices. Check connection and config.")

    def get_frame(self, is_reverse: bool):
        cam_type = 'reverse' if is_reverse else 'forward'
        
        if cam_type not in self.pipelines:
            logging.warning(f"Pipeline for '{cam_type}' camera not available.")
            return None
            
        pipeline = self.pipelines[cam_type]
        try:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                logging.warning(f"No color frame received from {cam_type} camera.")
                return None
            return np.asanyarray(color_frame.get_data())
        except Exception as e:
            logging.error(f"Error getting frame from {cam_type} camera: {e}")
            return None

    def stop(self):
        logging.info("Stopping RealSense cameras.")
        for cam_type, pipeline in self.pipelines.items():
            try:
                pipeline.stop()
                logging.info(f"Pipeline for {cam_type} camera stopped.")
            except Exception as e:
                logging.error(f"Error stopping {cam_type} pipeline: {e}")
        self.pipelines.clear()

# --- StanleyController, LaneDetector, Visualizer (Original) ---
class StanleyController:
    """Calculates steering angle using the Stanley control method."""
    @staticmethod
    def calculate_steering(cross_track_error, heading_error, speed, k, epsilon, is_reverse):
        cross_track_term = math.atan2(k * cross_track_error, abs(speed) + epsilon)
        return heading_error + cross_track_term
    
    @staticmethod
    def calculate_velocity(cross_track_error, heading_error, speed, k_cte, k_heading, is_reverse):
        """Calculate the desired velocity based on cross track error and heading error."""
        speed_factor = 1.0 / (1 + k_cte * abs(cross_track_error) + k_heading * abs(heading_error))
        desired_speed = speed * speed_factor
        if is_reverse:
            desired_speed *= -1
        return desired_speed

class LaneDetector:
    """Handles the full CV pipeline to detect lane and calculate errors using GPU."""
    def __init__(self, config: dict, use_cuda: bool):
        self.config = config
        self.ld_config = config['lane_detection']
        self.use_cuda = use_cuda
        if self.use_cuda:
            self.gpu_frame = cv2.cuda_GpuMat()
            self.gpu_warped = cv2.cuda_GpuMat()
            self.gpu_hsv = cv2.cuda_GpuMat()
            self.gpu_mask = cv2.cuda_GpuMat()
            logging.info("LaneDetector initialized with CUDA support.")
        else:
            logging.info("LaneDetector initialized for CPU processing.")

    def _get_perspective_transform(self, img_shape):
        H, W = img_shape
        warp = self.ld_config['perspective_warp']
        mirror_point = 1 - warp
        src = np.float32([[W * warp, H], [0, 0], [W, 0], [W * mirror_point, H]])
        dst = np.float32([[0, H], [0, 0], [W, 0], [W, H]])
        return cv2.getPerspectiveTransform(dst, src)

    def _find_lane_contours(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        centers = []
        min_area = self.ld_config['min_contour_area']
        max_area = self.ld_config.get('max_contour_area', float('inf'))
        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > min_area and M['m00'] < max_area:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append({'x': cx, 'y': cy, 'area': M['m00']})
        return contours, centers

    @staticmethod
    def _region_of_interest(img, vertices):
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, vertices, 255)
        return cv2.bitwise_and(img, mask)

    def _calculate_perpendicular_error(self, m, b, H, W):
        pixels_per_meter = 1.0 / self.ld_config.get('pixels_to_meters', 0.0035) 
        axle_offset_m = self.ld_config.get('axle_to_bottom_frame_m', 0.0)
        y_offset_pixels = int(axle_offset_m * pixels_per_meter)
        virtual_axle_point = (W // 2, H - y_offset_pixels)
        
        if abs(m) < 1e-6:
            x_int, y_int = virtual_axle_point[0], b
        else:
            perp_m = -1 / m
            perp_b = virtual_axle_point[1] - perp_m * virtual_axle_point[0]
            x_int = (perp_b - b) / (m - perp_m)
            y_int = m * x_int + b
            
        intersection_point = (int(x_int), int(y_int))
        error_pixels = math.hypot(x_int - virtual_axle_point[0], y_int - virtual_axle_point[1])
        
        virtual_axle_y_coord = H - y_offset_pixels
        x_at_axle_line = (virtual_axle_y_coord - b) / m if abs(m) > 1e-6 else float('inf')
        
        if x_at_axle_line < W // 2:
            error_pixels = -error_pixels
            
        return error_pixels, intersection_point

    def process_frame(self, img):
        H, W = img.shape[:2]
        M = self._get_perspective_transform((H, W))

        if self.use_cuda:
            self.gpu_frame.upload(img)
            self.gpu_warped = cv2.cuda.warpPerspective(self.gpu_frame, M, (W, H), flags=cv2.INTER_LINEAR)
            self.gpu_hsv = cv2.cuda.cvtColor(self.gpu_warped, cv2.COLOR_BGR2HSV)
            self.gpu_mask = cv2.cuda.inRange(self.gpu_hsv, tuple(self.ld_config['hsv_lower']), tuple(self.ld_config['hsv_upper']))
            mask_yellow = self.gpu_mask.download()
            img_warped = self.gpu_warped.download()
        else:
            img_warped = cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_LINEAR)
            hsv_img = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(hsv_img, np.array(self.ld_config['hsv_lower']), np.array(self.ld_config['hsv_upper']))

        all_contours, all_centers = [], []
        num_segments = self.ld_config['segments']
        region_height = H // num_segments
        for i in range(num_segments):
            segment_vertices = np.array([[(0, H - (i + 1) * region_height), (W, H - (i + 1) * region_height),
                                          (W, H - i * region_height), (0, H - i * region_height)]], dtype=np.int32)
            segment_mask = self._region_of_interest(mask_yellow, segment_vertices)
            contours, centers = self._find_lane_contours(segment_mask)
            all_contours.extend(contours)
            all_centers.extend(centers)

        centers_sorted = sorted(all_centers, key=lambda c: -c['y'])
        pipeline_data = {'warped_image': img_warped, 'color_mask': mask_yellow, 'contours': all_contours,
                         'centers': all_centers, 'cross_track_error': 0.0, 'heading_error': 0.0,
                         'lane_points': [], 'cte_intersection_point': None}

        if len(centers_sorted) >= 2:
            p1 = (centers_sorted[0]['x'], centers_sorted[0]['y'])
            p2 = (centers_sorted[1]['x'], centers_sorted[1]['y'])
            pipeline_data['lane_points'] = [p1, p2]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]

            if abs(dx) > 1e-6:
                m = dy / dx
                b = p1[1] - m * p1[0]
                pipeline_data['heading_error'] = (math.pi / 2) - math.atan2(-dy, dx)
                error_pixels, intersection_point = self._calculate_perpendicular_error(m, b, H, W)
                pipeline_data['cross_track_error'] = error_pixels * self.ld_config['pixels_to_meters']
                pipeline_data['cte_intersection_point'] = intersection_point
        else:
            logging.debug("Not enough lane points detected to calculate errors.")

        return pipeline_data

class Visualizer:
    """Handles all OpenCV drawing operations."""
    def draw_pipeline_data(self, image, data, steering_angle, current_speed, desired_speed):
        H, W = image.shape[:2]
        cv2.drawContours(image, data['contours'], -1, (255, 0, 255), 1)
        for center in data['centers']: 
            cv2.circle(image, (center['x'], center['y']), 5, (0, 255, 0), -1)
        if data['lane_points']: 
            cv2.line(image, data['lane_points'][0], data['lane_points'][1], (255, 0, 0), 3)
        if data['cte_intersection_point']:
            bottom_center = (W // 2, H)
            cv2.line(image, bottom_center, data['cte_intersection_point'], (255, 0, 255), 2)
        
        arrow_length = 100
        x0, y0 = W // 2, H
        arrow_angle = (math.pi / 2) - steering_angle
        x1, y1 = int(x0 + arrow_length * math.cos(arrow_angle)), int(y0 - arrow_length * math.sin(arrow_angle))
        cv2.arrowedLine(image, (x0, y0), (x1, y1), (0, 0, 255), 3, tipLength=0.2)
        
        cv2.putText(image, f"CTE: {data['cross_track_error']:.3f} m", (10, H - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"HE: {math.degrees(data['heading_error']):.1f} deg", (10, H - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"Steer: {math.degrees(steering_angle):.1f} deg", (10, H - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        cv2.putText(image, f"Speed: {current_speed:.1f} rpm", (10, H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(image, f"Target: {desired_speed:.1f} rpm", (10, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        return image

class LineFollowingNode(ManagedNode, ConfigMixin):
    """A managed node for running the line following vision pipeline using dual RealSense cameras."""
    
    def __init__(self, node_name: str = "line_follower_node", config_path: str = "config.yaml"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        # --- Atribut Asli ---
        self.camera_handler = None
        self.lane_detector = None
        self.visualizer = None
        self.pub_socket = None
        self.hmi_sub_socket = None
        self.sensor_sub_socket = None
        self.result_writer = None
        self.frame_pub_socket = None
        self.is_reverse = False
        self.current_speed_rpm = 0.0
        self.last_sensor_update = 0
        self.rpm_to_ms = 0.0
        self.vehicle_params = None
        
        self.processing_thread = None
        self.active_event = threading.Event()

        # --- PENAMBAHAN DIMULAI ---
        # Atribut untuk fungsionalitas Aruco
        self.aruco_detector = None
        self.distance_pub = None
        self.forward_camera_matrix = None
        self.forward_dist_coeffs = None
        self.reverse_camera_matrix = None
        self.reverse_dist_coeffs = None
        self.active_camera_matrix = None
        self.active_dist_coeffs = None
        # --- PENAMBAHAN SELESAI ---

    def on_configure(self) -> bool:
        """Load configuration and initialize all components."""
        self.logger.info("Configuring Line Following Node...")
        try:
            self.vehicle_params = self.get_section_config('vehicle_params')
            
            # Steering command publisher
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(self.get_zmq_url('steering_cmd_url'))
            
            # HMI command subscriber
            self.hmi_sub_socket = self.context.socket(zmq.SUB)
            self.hmi_sub_socket.connect(self.get_zmq_url('hmi_cmd_url'))
            self.hmi_sub_socket.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('hmi_direction_topic'))

            # Sensor data subscriber
            self.sensor_sub_socket = self.context.socket(zmq.SUB)
            self.sensor_sub_socket.connect(self.get_zmq_url('sensor_data_url'))
            self.sensor_sub_socket.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('sensor_data_topic'))
            
            # --- Direct RealSense Camera Setup ---
            rs_cam_config = self.get_section_config('realsense_cameras')
            self.camera_handler = MultiRealSenseCamera(rs_cam_config)
            
            use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
            self.logger.info(f"CUDA Available: {use_cuda}. {'Using GPU.' if use_cuda else 'Running on CPU.'}")
            self.lane_detector = LaneDetector(self.config, use_cuda)
            self.visualizer = Visualizer()

            # --- PENAMBAHAN DIMULAI ---
            # Konfigurasi Aruco
            self.logger.info("Configuring Aruco detector...")
            aruco_config = self.get_section_config('aruco')

            # Muat file kalibrasi FORWARD
            forward_calib_file = aruco_config['forward_calibration_file']
            fs_forward = cv2.FileStorage(forward_calib_file, cv2.FILE_STORAGE_READ)
            if not fs_forward.isOpened():
                raise IOError(f"Failed to open FORWARD calibration file: {forward_calib_file}")
            self.forward_camera_matrix = fs_forward.getNode("camera_matrix").mat()
            self.forward_dist_coeffs = fs_forward.getNode("dist_coeff").mat()
            fs_forward.release()
            self.logger.info(f"Successfully loaded FORWARD calibration from {forward_calib_file}")

            # Muat file kalibrasi REVERSE
            reverse_calib_file = aruco_config['reverse_calibration_file']
            fs_reverse = cv2.FileStorage(reverse_calib_file, cv2.FILE_STORAGE_READ)
            if not fs_reverse.isOpened():
                raise IOError(f"Failed to open REVERSE calibration file: {reverse_calib_file}")
            self.reverse_camera_matrix = fs_reverse.getNode("camera_matrix").mat()
            self.reverse_dist_coeffs = fs_reverse.getNode("dist_coeff").mat()
            fs_reverse.release()
            self.logger.info(f"Successfully loaded REVERSE calibration from {reverse_calib_file}")

            # Inisialisasi Aruco detector
            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
            # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
            # parameters = cv2.aruco.DetectorParameters()
            parameters = aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            
            # Buat publisher untuk data jarak Aruco
            self.distance_pub = self.context.socket(zmq.PUB)
            self.distance_pub.bind(self.get_zmq_url('distance_url'))
            self.logger.info(f"Aruco distance publisher bound to {self.get_zmq_url('distance_url')}")
            # --- PENAMBAHAN SELESAI ---
            
            # Conditional video writer
            if self.vehicle_params.get('enable_video_recording', False):
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                video_filename = f"video_output_{timestamp}.mp4"
                self.result_writer = cv2.VideoWriter(
                    video_filename, fourcc, rs_cam_config['frame_fps'],
                    (rs_cam_config['frame_width'] * 2, rs_cam_config['frame_height'])
                )
                self.logger.info(f"Video writer enabled. Saving results to {video_filename}")
            
            # Conditional frame publisher (for debugging)
            if self.vehicle_params.get('enable_frame_publishing', False):
                self.frame_pub_socket = self.context.socket(zmq.PUB)
                publish_url = self.get_zmq_url('frame_publish_url')
                self.frame_pub_socket.bind(publish_url)
                self.logger.info(f"Frame publishing enabled. Broadcasting on {publish_url}")
            else:
                self.logger.info("Frame publishing is disabled.")

            self.logger.info("Line Following Node configuration successful.")
            return True
            
        except Exception as e:
            self.logger.exception(f"Error during configuration: {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating Line Following Node...")
        try:
            self.camera_handler.start()
            self.active_event.set() 
            self.processing_thread = threading.Thread(
                target=self._processing_loop, 
                name=f"{self.node_name}_ProcessingThread"
            )
            self.processing_thread.start()
            self.logger.info("Line Following Node activated.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during activation: {e}")
            return False

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating Line Following Node...")
        try:
            self.active_event.clear()
            if self.processing_thread:
                self.processing_thread.join(timeout=2.0)
            if self.camera_handler:
                self.camera_handler.stop()
            self.logger.info("Line Following Node deactivated.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during deactivation: {e}")
            return False

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down Line Following Node...")
        if self.state == "active":
            self.on_deactivate()
            
        try:
            if self.result_writer:
                self.result_writer.release()
            if self.pub_socket:
                self.pub_socket.close()
            if self.hmi_sub_socket:
                self.hmi_sub_socket.close()
            if self.sensor_sub_socket:
                self.sensor_sub_socket.close()
            if self.frame_pub_socket:
                self.frame_pub_socket.close()
            
            # --- PENAMBAHAN DIMULAI ---
            # Tutup socket publisher jarak Aruco
            if self.distance_pub:
                self.distance_pub.close()
            # --- PENAMBAHAN SELESAI ---

            if self.context:
                self.context.term()

            cv2.destroyAllWindows()
            self.logger.info("Line Following Node shutdown successful.")
            return True
        except Exception as e:
            self.logger.exception(f"Error during shutdown: {e}")
            return False
            
    def _processing_loop(self):
        """Main processing loop for line following."""
        start_time = time.time()
        frame_count = 0
        rs_cam_config = self.get_section_config('realsense_cameras')
        sc_config = self.get_section_config('stanley_controller')

        # --- PENAMBAHAN DIMULAI ---
        # Ambil config Aruco di awal loop
        aruco_config = self.get_section_config('aruco')
        # --- PENAMBAHAN SELESAI ---

        # Poller for asynchronous ZMQ messages (HMI, sensors)
        poller = zmq.Poller()
        poller.register(self.hmi_sub_socket, zmq.POLLIN)
        poller.register(self.sensor_sub_socket, zmq.POLLIN)

        while self.active_event.is_set() and not self.shutdown_event.is_set():
            # Poll for ZMQ messages with a short timeout to not block frame acquisition
            socks = dict(poller.poll(timeout=10))

            if self.hmi_sub_socket in socks:
                topic, msg = self.hmi_sub_socket.recv_multipart()
                self.is_reverse = msg.decode('utf-8') == 'reverse'
                self.logger.info(f"Direction changed to {'REVERSE' if self.is_reverse else 'FORWARD'}")

            if self.sensor_sub_socket in socks:
                topic, sensor_data_json = self.sensor_sub_socket.recv_multipart()
                try:
                    sensor_data = json.loads(sensor_data_json.decode('utf-8'))
                    self.current_speed_rpm = sensor_data.get('rpm', 0.0)
                    self.last_sensor_update = time.time()
                except (json.JSONDecodeError, KeyError) as e:
                    self.logger.warning(f"Failed to parse sensor data: {e}")
            
            # Get frame from the appropriate camera. This is a blocking call.
            frame = self.camera_handler.get_frame(is_reverse=self.is_reverse)
            if frame is None:
                self.logger.warning("Failed to get frame. Skipping processing cycle.")
                time.sleep(0.1) # Avoid busy-looping if cameras disconnect
                continue
            
            # --- TUGAS 1: LANE FOLLOWING (Logika Asli) ---
            lane_data = self.lane_detector.process_frame(frame)

            current_speed_ms = abs(self.current_speed_rpm) * self.vehicle_params['rpm_to_mps_factor']
            
            desired_speed_ms = StanleyController.calculate_velocity(
                lane_data['cross_track_error'], lane_data['heading_error'],
                sc_config.get('max_speed', 1.0), sc_config.get('velocity_k_cte', 2.0),
                sc_config.get('velocity_k_heading', 1.0), self.is_reverse
            )

            steering_angle_rad = StanleyController.calculate_steering(
                lane_data['cross_track_error'], lane_data['heading_error'],
                desired_speed_ms, sc_config['gain'], sc_config['speed_epsilon'], self.is_reverse
            )

            if self.is_reverse:
                steering_angle_rad *= -1
            
            desired_speed_rpm = desired_speed_ms / self.vehicle_params['rpm_to_mps_factor']

            steering_angle_deg = math.degrees(steering_angle_rad)

            command = steering_command_pb2.SteeringCommand()
            command.auto_steer_angle = steering_angle_deg
            command.speed = desired_speed_rpm
            serialized_command = command.SerializeToString()
            self.pub_socket.send_string(self.get_zmq_topic('steering_cmd_topic'), flags=zmq.SNDMORE)
            self.pub_socket.send(serialized_command)

            # --- PENAMBAHAN DIMULAI ---
            # --- TUGAS 2: DETEKSI JARAK ARUCO ---
            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
            if ids is not None:
                # Pilih data kalibrasi yang benar berdasarkan arah
                if self.is_reverse:
                    self.active_camera_matrix = self.reverse_camera_matrix
                    self.active_dist_coeffs = self.reverse_dist_coeffs
                else:
                    self.active_camera_matrix = self.forward_camera_matrix
                    self.active_dist_coeffs = self.forward_dist_coeffs

                # Lakukan estimasi pose dengan data kalibrasi yang aktif
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, aruco_config['known_marker_width_m'], self.active_camera_matrix, self.active_dist_coeffs
                )
                direct_dist = tvecs[0][0][2]
                
                if direct_dist > aruco_config['camera_height_m']:
                    ground_distance = math.sqrt(direct_dist**2 - aruco_config['camera_height_m']**2)
                    
                    # Publikasikan jarak
                    self.distance_pub.send_string(self.get_zmq_topic('distance_topic'), flags=zmq.SNDMORE)
                    self.distance_pub.send_string(f"{ground_distance}")

                    # Gambar visualisasi Aruco pada frame original
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(frame, self.active_camera_matrix, self.active_dist_coeffs, rvecs[0], tvecs[0], 0.1)
                    cv2.putText(frame, f"Aruco Dist: {ground_distance:.2f} m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            # --- PENAMBAHAN SELESAI ---

            # --- Visualisasi Gabungan (Logika Asli) ---
            warped_view = lane_data.get('warped_image', np.zeros_like(frame))
            annotated_warped = self.visualizer.draw_pipeline_data(
                warped_view, lane_data, steering_angle_rad, 
                self.current_speed_rpm, desired_speed_rpm
            )
            stacked_output = np.hstack((frame, annotated_warped))
            
            # Use general display setting from vehicle_params
            if self.vehicle_params.get('use_display', False):
                cv2.imshow('Lane Following View', stacked_output)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.shutdown_event.set()
                    break
            
            if self.result_writer:
                self.result_writer.write(stacked_output)

            if self.frame_pub_socket:
                try:
                    frame_msg = frame_data_pb2.FrameData()
                    frame_msg.frame = stacked_output.tobytes()
                    frame_msg.height = stacked_output.shape[0]
                    frame_msg.width = stacked_output.shape[1]
                    frame_msg.channels = stacked_output.shape[2]
                    
                    serialized_frame = frame_msg.SerializeToString()
                    self.frame_pub_socket.send_multipart([
                        self.get_zmq_topic('frame_publish_topic').encode('utf-8'),
                        serialized_frame
                    ])
                except Exception as e:
                    self.logger.error(f"Failed to publish frame: {e}")

            frame_count += 1
            if frame_count % 30 == 0:
                elapsed_time = time.time() - start_time
                fps = frame_count / elapsed_time
                self.logger.info(f"Processing FPS: {fps:.2f}")
        
        self.logger.info("Line Following processing loop terminated.")

def main():
    setup_logging()
    line_follower = LineFollowingNode()
    line_follower.run()

if __name__ == "__main__":
    main()