import zmq
import time
import json
import logging
import threading
from managednode import ManagedNode
from shared_enums import NodeState
from config_mixin import ConfigMixin
import steering_command_pb2
import obstacle_data_pb2

# --- Konfigurasi TCP (Sesuai Original) ---
ZMQ_HMI_SUB_URL = "tcp://localhost:5557"
HMI_TOPIC = "hmi_cmd"
ZMQ_STEER_SUB_URL = "ipc:///tmp/teleop_cmd.ipc"
LANE_ASSIST_TOPIC = "lane_assist_angle"
ZMQ_LLC_PUB_URL = "tcp://localhost:5560"
LLC_TOPIC = "teleop_cmd"
ZMQ_DISTANCE_SUB_URL = "tcp://localhost:5556"
DISTANCE_TOPIC = "aruco_distance"

# --- Parameter Kendaraan (Sesuai Original) ---
MAX_SPEED_RPM = 1000.0
SPEED_RAMP_RATE = 500
BRAKE_RAMP_RATE = 120
MAX_BRAKE_FORCE = 253
DISTANCE_THRESHOLD = 1.7

# --- PENAMBAHAN DIMULAI ---
# Konstanta untuk timeout data Aruco
ARUCO_DATA_TIMEOUT = 2.0  # Detik
# --- PENAMBAHAN SELESAI ---

class ControlNode(ManagedNode, ConfigMixin):
    def __init__(self, node_name="control_node", config_path="config.yaml"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        # Variabel socket sesuai original
        self.hmi_sub = None
        self.hmi_sub_direction = None
        self.steer_sub = None
        self.distance_sub = None
        self.llc_pub = None
        self.control_poller = zmq.Poller()

        # Variabel status sesuai original
        self.is_running = False
        self.is_reverse = False
        self.current_speed_rpm = 0.0
        self.current_steer_angle = 0.0
        self.desired_speed_rpm = 0.0
        self.time_stopped = None
        self.time_started = None
        self.processing_thread = None
        self.active_event = threading.Event()
        self.last_log_time = 0
        self.log_interval = 1.0
        self.last_received_distance = None

        # --- PENAMBAHAN DIMULAI ---
        # Variabel untuk manajemen status halangan Aruco
        self.obstacle_detected = False
        self.last_aruco_update_time = 0
        # --- PENAMBAHAN SELESAI ---

    def on_configure(self) -> bool:
        self.logger.info("Configuring Control Node...")
        try:
            # Setup ZMQ sesuai original (dengan 2 socket HMI)
            self.hmi_sub = self.context.socket(zmq.SUB)
            self.hmi_sub.connect(self.get_zmq_url('hmi_cmd_url'))
            self.hmi_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('hmi_cmd_topic'))

            self.hmi_sub_direction = self.context.socket(zmq.SUB)
            self.hmi_sub_direction.connect(self.get_zmq_url('hmi_cmd_url'))
            self.hmi_sub_direction.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('hmi_direction_topic'))

            self.steer_sub = self.context.socket(zmq.SUB)
            self.steer_sub.connect(self.get_zmq_url('steering_cmd_url'))
            self.steer_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('steering_cmd_topic'))
            
            self.distance_sub = self.context.socket(zmq.SUB)
            self.distance_sub.connect(self.get_zmq_url('distance_url'))
            self.distance_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('distance_topic'))

            self.llc_pub = self.context.socket(zmq.PUB)
            self.llc_pub.bind(self.get_zmq_url('control_cmd_url'))

            # Register semua socket ke poller sesuai original
            self.control_poller.register(self.hmi_sub, zmq.POLLIN)
            self.control_poller.register(self.hmi_sub_direction, zmq.POLLIN)
            self.control_poller.register(self.steer_sub, zmq.POLLIN)
            self.control_poller.register(self.distance_sub, zmq.POLLIN)
            
                   # --- Tambah SUB untuk LIDAR safety zone ---
            self.lidar_sub = self.context.socket(zmq.SUB)
            self.lidar_sub.connect(self.get_zmq_url('obstacle_data_sub'))
            self.lidar_sub.setsockopt_string(zmq.SUBSCRIBE,
                                            self.get_zmq_topic('obstacle_data_sub_topic'))
            self.control_poller.register(self.lidar_sub, zmq.POLLIN)

            # Baca batas safety zone dari config
            sz = self.config['safety_zone']
            self.min_x = float(sz['min_x'])
            self.max_x = float(sz['max_x'])
            self.min_y = float(sz['min_y'])
            self.max_y = float(sz['max_y'])
            self.logger.info(f"Safety zone loaded: x[{self.min_x},{self.max_x}] "
                            f"y[{self.min_y},{self.max_y}]")

            return True
        except zmq.ZMQError as e:
            self.logger.error(f"ZMQ Error: {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating Control Node...")
        try:
            self.active_event.set()
            self.processing_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.processing_thread.start()
        except Exception as e:
            self.logger.error(f"Activation failed: {e}")
            return False
        return True

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating Control Node...")
        try:
            self.active_event.clear()
            if self.processing_thread:
                self.processing_thread.join(timeout=1.0)
            self.logger.info("Control Node deactivated.")
            return True
        except Exception as e:
            self.logger.error(f"Deactivation failed: {e}")
            return False

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down Control Node...")
        try:
            if self.state == NodeState.ACTIVE: self.on_deactivate()
            if self.hmi_sub: self.hmi_sub.close()
            # Pastikan semua socket ditutup
            if self.hmi_sub_direction: self.hmi_sub_direction.close()
            if self.steer_sub: self.steer_sub.close()
            if self.distance_sub: self.distance_sub.close()
            if self.llc_pub: self.llc_pub.close()
            return True
        except Exception as e:
            self.logger.error(f"Error during shutdown: {e}")
            return False

    def _control_loop(self):
        self.logger.info("Control loop started.")
        
        while self.active_event.is_set():
            # --- PENAMBAHAN DIMULAI ---
            # Logika untuk menangani timeout data Aruco, jika halangan tidak terlihat lagi
            if self.obstacle_detected and (time.time() - self.last_aruco_update_time > ARUCO_DATA_TIMEOUT):
                self.logger.warning("Aruco data timed out. Assuming obstacle is clear.")
                self.obstacle_detected = False
            # --- PENAMBAHAN SELESAI ---

            socks = dict(self.control_poller.poll(100))

            # Logika HMI Direction original
            if self.hmi_sub_direction in socks:
                topic, msg = self.hmi_sub_direction.recv_multipart()
                direction = msg.decode('utf-8')
                self.logger.info(f"Received HMI direction command on topic {topic.decode('utf-8')}: {direction}")
                if direction == "reverse":
                    self.is_reverse = True
                elif direction == "forward":
                    self.is_reverse = False
                else:
                    self.logger.warning(f"Unknown direction command: {direction}")
            
            # Logika HMI Command original
            if self.hmi_sub in socks:
                topic, msg = self.hmi_sub.recv_multipart()
                command = msg.decode('utf-8')
                self.logger.info(f"Received HMI command on topic {topic.decode('utf-8')}: {command}")
                if command == "START" and not self.is_running:
                    self.is_running = True
                    self.time_stopped = None
                    self.time_started = time.time()
                    self.logger.info("START command received. Vehicle moving.")
                elif command == "STOP" and self.is_running:
                    self.is_running = False
                    self.time_stopped = time.time()
                    self.time_started = None
                    self.logger.info("STOP command received. Vehicle stopping.")
                elif "REV" in command:
                    rev_state = bool(int(command[3]))
                    self.logger.info(f"Reverse is {'on' if rev_state else 'off'} from HMI.")
                    self.is_reverse = rev_state

            # Logika Logging original
            current_time = time.time()
            if (current_time - self.last_log_time) >= self.log_interval:
                if self.last_received_distance is not None:
                    distance = self.last_received_distance
                    log_message = f"DATA: Jarak terakhir {distance:.2f} m. "
                    if distance < DISTANCE_THRESHOLD:
                        self.logger.warning(log_message)
                    else:
                        self.logger.info(log_message)
                    self.last_received_distance = None
                else:
                    self.logger.info("STATUS: Tidak ada data jarak baru yang masuk.")

                self.logger.info(f"STATUS: Kendaraan {'berjalan' if self.is_running else 'berhenti'}, "
                                 f"kecepatan {self.current_speed_rpm:.2f} RPM, "
                                 f"steer angle {self.current_steer_angle:.2f} derajat, "
                                 f"reverse {'aktif' if self.is_reverse else 'non-aktif'}.")
                self.last_log_time = current_time
            
            # Logika Distance (Aruco) original + penambahan
            if self.distance_sub in socks:
                topic, msg = self.distance_sub.recv_multipart()
                # Baris asli untuk logging dipertahankan
                self.last_received_distance = float(msg.decode('utf-8'))
                
                # --- PENAMBAHAN DIMULAI ---
                # Manajemen status halangan
                self.last_aruco_update_time = time.time()
                distance = self.last_received_distance # gunakan nilai yang sama
                self.logger.info(f"Received distance data on topic {topic.decode('utf-8')}: {distance:.2f} m")
                if distance < DISTANCE_THRESHOLD:
                    if not self.obstacle_detected:
                        self.logger.warning(f"HALANGAN ARUCO: Terdeteksi pada {distance:.2f} m. Memaksa berhenti!")
                    self.obstacle_detected = True
                # --- PENAMBAHAN SELESAI ---

            # --- PENYESUAIAN KONDISI BERHENTI ---
            # Kondisi berhenti karena Aruco kini menggunakan status 'obstacle_detected'
            if self.obstacle_detected and self.is_running:
                self.is_running = False
                self.time_stopped = time.time()
                self.time_started = None
                self.logger.info(f"Berhenti darurat karena halangan Aruco. Status berjalan: {self.is_running}")

            # --- Cek data LIDAR untuk safety zone ---
            if self.lidar_sub in socks:
                _, serialized = self.lidar_sub.recv_multipart()
                scan = obstacle_data_pb2.LidarScan()
                scan.ParseFromString(serialized)
                for pt in scan.points:
                    x, y = pt.x_m, pt.y_m
                    if self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y:
                        # self.logger.warning(f"[SAFETY] Obstacle detected in safety zone at ({x:.2f}, {y:.2f})!")
                        # Tidak aman: hentikan kendaraan
                        if self.is_running:
                            print(f"[SAFETY] Obstacle di ({x:.2f}, {y:.2f})! STOP!")
                            self.is_running = False
                            self.time_stopped = time.time()
                            self.time_started = None
                        break

            # Logika Steering original
            if self.steer_sub in socks:
                topic, serialized_data = self.steer_sub.recv_multipart()
                command = steering_command_pb2.SteeringCommand()
                command.ParseFromString(serialized_data)
                self.current_steer_angle = command.auto_steer_angle
                self.desired_speed_rpm = command.speed

            # Logika Kecepatan dan Rem original
            brake_force = 0
            if self.is_running:
                if self.time_started is not None:
                    elapsed_time = time.time() - self.time_started
                    self.current_speed_rpm = min(abs(self.desired_speed_rpm), elapsed_time * SPEED_RAMP_RATE)
                    if self.is_reverse:
                        self.current_speed_rpm *= -1
                    # self.logger.info(f"Current speed: {self.current_speed_rpm:.2f} RPM; elapsed time: {elapsed_time:.2f}. seconds")
            else:
                self.current_speed_rpm = 0
                if self.time_stopped is not None:
                    elapsed_time = time.time() - self.time_stopped
                    brake_force = min(MAX_BRAKE_FORCE, int(elapsed_time * BRAKE_RAMP_RATE))
                    self.logger.info(f"Brake force applied: {brake_force} N; time stopped: {elapsed_time:.2f} seconds ago.")

            self._send_llc_command(self.current_speed_rpm, self.current_steer_angle, brake_force)
            time.sleep(0.02)
        self.logger.info("Control loop stopped.")
    
    def _send_llc_command(self, speed, steer, brake):
        command = {"speed_rpm": speed, "steer_angle": steer, "brake_force": brake}
        self.llc_pub.send_string(self.get_zmq_topic('control_cmd_topic'), flags=zmq.SNDMORE)
        self.llc_pub.send_json(command)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    control_node = ControlNode()
    control_node.run()
