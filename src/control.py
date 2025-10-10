import zmq
import time
import math
import logging
import threading
import ast
from managednode import ManagedNode
from shared_enums import NodeState
from config_mixin import ConfigMixin
import steering_command_pb2

# --- Konstanta ---
ARUCO_DATA_TIMEOUT = 2.0
DISTANCE_THRESHOLD = 1.7
SPEED_RAMP_RATE = 500
MAX_BRAKE_FORCE = 252
BRAKE_LOG_BASE = 10.0
BRAKE_SCALING_FACTOR = 252.0

class ControlNode(ManagedNode, ConfigMixin):
    def __init__(self, node_name="control_node", config_path="config.yaml"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        # Sockets
        self.hmi_sub = None
        self.steer_sub = None
        self.distance_sub = None
        self.llc_pub = None
        self.direction_pub = None
        self.lidar_cmd_sub = None
        self.control_poller = zmq.Poller()

        # State
        self.is_running = False
        self.is_reverse = False
        self.current_speed_rpm = 0.0
        self.current_steer_angle = 0.0
        self.desired_speed_rpm = 0.0
        self.time_stopped = None
        self.time_started = None
        
        # Obstacle States
        self.aruco_obstacle = False
        self.lidar_command = "GO"
        self.last_aruco_update_time = 0
        # --- DITAMBAHKAN: Variabel untuk menyimpan ID ArUco pemicu ---
        self.triggering_aruco_id = None
        
        # Threading
        self.processing_thread = None
        self.active_event = threading.Event()

    def on_configure(self) -> bool:
        self.logger.info("Configuring Control Node...")
        try:
            # HMI and Vehicle Sockets
            self.hmi_sub = self.context.socket(zmq.SUB)
            self.hmi_sub.connect(self.get_zmq_url('hmi_cmd_url'))
            self.hmi_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('hmi_cmd_topic'))
            self.hmi_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('hmi_direction_topic'))

            self.steer_sub = self.context.socket(zmq.SUB)
            self.steer_sub.connect(self.get_zmq_url('steering_cmd_url'))
            self.steer_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('steering_cmd_topic'))
            
            self.distance_sub = self.context.socket(zmq.SUB)
            self.distance_sub.connect(self.get_zmq_url('distance_url'))
            self.distance_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('distance_topic'))

            self.llc_pub = self.context.socket(zmq.PUB)
            self.llc_pub.bind(self.get_zmq_url('control_cmd_url'))

            # Sockets for LIDAR Communication
            self.direction_pub = self.context.socket(zmq.PUB)
            self.direction_pub.bind(self.get_zmq_url('direction_status_url'))

            self.lidar_cmd_sub = self.context.socket(zmq.SUB)
            self.lidar_cmd_sub.connect(self.get_zmq_url('lidar_command_url'))
            self.lidar_cmd_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('lidar_command_topic'))

            # Register all sockets to poller
            self.control_poller.register(self.hmi_sub, zmq.POLLIN)
            self.control_poller.register(self.steer_sub, zmq.POLLIN)
            self.control_poller.register(self.distance_sub, zmq.POLLIN)
            self.control_poller.register(self.lidar_cmd_sub, zmq.POLLIN)
            return True
        except zmq.ZMQError as e:
            self.logger.error(f"ZMQ Error during configuration: {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating Control Node...")
        self.active_event.set()
        self.processing_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.processing_thread.start()
        return True

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating Control Node...")
        self.active_event.clear()
        if self.processing_thread:
            self.processing_thread.join(timeout=1.0)
        return True

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down Control Node...")
        if self.state == NodeState.ACTIVE: self.on_deactivate()
        if self.hmi_sub: self.hmi_sub.close()
        if self.steer_sub: self.steer_sub.close()
        if self.distance_sub: self.distance_sub.close()
        if self.llc_pub: self.llc_pub.close()
        if self.direction_pub: self.direction_pub.close()
        if self.lidar_cmd_sub: self.lidar_cmd_sub.close()
        return True

    def _publish_direction(self):
        direction_str = "REVERSE" if self.is_reverse else "FORWARD"
        self.direction_pub.send_string(self.get_zmq_topic('direction_status_topic'), flags=zmq.SNDMORE)
        self.direction_pub.send_string(direction_str)

    def _control_loop(self):
        self.logger.info("Control loop started.")
        last_direction_publish_time = 0
        
        while self.active_event.is_set():
            if time.time() - last_direction_publish_time > 0.5:
                self._publish_direction()
                last_direction_publish_time = time.time()

            if self.aruco_obstacle and (time.time() - self.last_aruco_update_time > ARUCO_DATA_TIMEOUT):
                self.logger.warning("Aruco data timed out. Assuming obstacle is clear.")
                self.aruco_obstacle = False
                # --- DITAMBAHKAN: Reset ID saat timeout ---
                self.triggering_aruco_id = None

            socks = dict(self.control_poller.poll(100))

            if self.hmi_sub in socks: self._handle_hmi_input()
            if self.distance_sub in socks: self._handle_distance_input()
            if self.steer_sub in socks: self._handle_steer_input()
            if self.lidar_cmd_sub in socks: self._handle_lidar_input()

            lidar_stop_relevant = \
                (not self.is_reverse and self.lidar_command == "STOP_DEPAN") or \
                (self.is_reverse and self.lidar_command == "STOP_BELAKANG")

            if self.aruco_obstacle or lidar_stop_relevant:
                if self.is_running:
                    # --- DIUBAH: Logika pesan peringatan ---
                    if self.aruco_obstacle:
                        reason = f"ARUCO (ID: {self.triggering_aruco_id})"
                    else:
                        reason = f"LIDAR ({self.lidar_command})"
                    # --- AKHIR PERUBAHAN ---
                    
                    self.logger.warning(f"EMERGENCY STOP triggered by: {reason}")
                    self.is_running = False
                    self.time_stopped = time.time()
                    self.time_started = None
            
            speed_reduction_factor = self.get_section_config('lidar_control')['speed_reduction_factor']
            brake_force = 0
            
            if self.is_running:
                effective_desired_speed = abs(self.desired_speed_rpm)
                
                lidar_reduce_relevant = \
                    (not self.is_reverse and self.lidar_command == "REDUCE_SPEED_DEPAN") or \
                    (self.is_reverse and self.lidar_command == "REDUCE_SPEED_BELAKANG")

                if lidar_reduce_relevant:
                    self.logger.info(f"Reducing speed due to relevant LIDAR command: {self.lidar_command}")
                    effective_desired_speed *= (1.0 - speed_reduction_factor)
                
                elapsed_time = time.time() - self.time_started
                self.current_speed_rpm = min(effective_desired_speed, elapsed_time * SPEED_RAMP_RATE)
                if self.is_reverse:
                    self.current_speed_rpm *= -1
            else:
                self.current_speed_rpm = 0
                if self.time_stopped is not None:
                    elapsed_time = time.time() - self.time_stopped
                    log_brake = BRAKE_SCALING_FACTOR * math.log(1 + elapsed_time, BRAKE_LOG_BASE)
                    brake_force = min(MAX_BRAKE_FORCE, int(log_brake))

            self._send_llc_command(self.current_speed_rpm, self.current_steer_angle, brake_force)
            time.sleep(0.02)

    def _handle_lidar_input(self):
        topic, msg = self.lidar_cmd_sub.recv_multipart()
        self.lidar_command = msg.decode('utf-8')
        self.logger.debug(f"Received LIDAR command: {self.lidar_command}")

    def _handle_hmi_input(self):
        topic_bytes, msg_bytes = self.hmi_sub.recv_multipart()
        topic = topic_bytes.decode('utf-8')
        command = msg_bytes.decode('utf-8')

        if topic == self.get_zmq_topic('hmi_direction_topic'):
            self.is_reverse = (command == "reverse")
        else:
            if command == "START" and not self.is_running:
                self.is_running = True
                self.time_stopped = None
                self.time_started = time.time()
            elif command == "STOP" and self.is_running:
                self.is_running = False
                self.time_stopped = time.time()
                self.time_started = None

    # =========================================================================
    # === FUNGSI INI DIUBAH UNTUK MENANGKAP ID ARUCO PENYEBAB STOP ===
    # =========================================================================
    def _handle_distance_input(self):
        topic_bytes, msg_bytes = self.distance_sub.recv_multipart()
        msg_string = msg_bytes.decode('utf-8')

        try:
            payload = ast.literal_eval(msg_string)
            aruco_data_list = payload.get('data', [])

            if not aruco_data_list:
                self.aruco_obstacle = False
                # --- DITAMBAHKAN: Reset ID saat tidak ada marker ---
                self.triggering_aruco_id = None
                return

            # --- DIUBAH: Cari marker terdekat (ID dan jaraknya) ---
            closest_marker = min(aruco_data_list, key=lambda item: item[1])
            trigger_id, min_distance = closest_marker
            # --- AKHIR PERUBAHAN ---

            self.last_aruco_update_time = time.time()

            if min_distance < DISTANCE_THRESHOLD:
                if not self.aruco_obstacle:
                    self.logger.warning(f"ARUCO obstacle detected at {min_distance:.2f} m.")
                self.aruco_obstacle = True
                # --- DITAMBAHKAN: Simpan ID pemicu ---
                self.triggering_aruco_id = trigger_id
            
        except (ValueError, SyntaxError) as e:
            self.logger.error(f"Failed to parse ArUco data: {msg_string}. Error: {e}")


    def _handle_steer_input(self):
        topic_bytes, steerangle_bytes, speed_rpm_bytes = self.steer_sub.recv_multipart()
        try:
            self.current_steer_angle = float(steerangle_bytes.decode('utf-8'))
            self.desired_speed_rpm = float(speed_rpm_bytes.decode('utf-8'))
        except ValueError as e:
            self.logger.error(f"Failed to parse steer input: {e}")

    def _send_llc_command(self, speed, steer, brake):
        command = {"speed_rpm": speed, "steer_angle": steer, "brake_force": brake}
        self.llc_pub.send_string(self.get_zmq_topic('control_cmd_topic'), flags=zmq.SNDMORE)
        self.llc_pub.send_json(command)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    control_node = ControlNode(config_path="../params/config.yaml")
    control_node.run()