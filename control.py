import zmq
import time
import json
import logging
import threading
from managednode import ManagedNode
from shared_enums import NodeState
import steering_command_pb2

# --- Konfigurasi TCP ---
ZMQ_HMI_SUB_URL = "tcp://localhost:5557"
HMI_TOPIC = "hmi_cmd"
ZMQ_STEER_SUB_URL = "ipc:///tmp/teleop_cmd.ipc"
ZMQ_STEER_SUB_URL = "ipc:///tmp/teleop_cmd_reverse.ipc"
LANE_ASSIST_TOPIC = "lane_assist_angle"
ZMQ_LLC_PUB_URL = "tcp://localhost:5560"
LLC_TOPIC = "teleop_cmd"
LLC_TOPIC_REVERSE = "teleop_cmd_reverse"
ZMQ_DISTANCE_SUB_URL = "tcp://localhost:5556"
DISTANCE_TOPIC = "aruco_distance"

# --- Parameter Kendaraan ---
MAX_SPEED_RPM = 1000.0
SPEED_RAMP_RATE = 500
BRAKE_RAMP_RATE = 50
MAX_BRAKE_FORCE = 100
DISTANCE_THRESHOLD = 0.2

class ControlNode(ManagedNode):
    # (Metode __init__ dan lainnya tidak berubah)
    def __init__(self, node_name="control_node"):
        super().__init__(node_name)
        self.hmi_sub, self.steer_sub, self.distance_sub, self.llc_pub = None, None, None, None
        self.control_poller = zmq.Poller()
        self.is_running = False
        self.is_reverse = False
        self.current_speed_rpm = 0.0
        self.current_steer_angle = 0.0
        self.time_stopped = None
        self.time_started = None

        self.processing_thread = None
        self.active_event = threading.Event()
        self.last_log_time = 0
        self.log_interval = 1.0
        self.last_received_distance = None

    def on_configure(self) -> bool:
        self.logger.info("Configuring Control Node...")
        try:
            self.hmi_sub = self.context.socket(zmq.SUB)
            self.hmi_sub.connect(ZMQ_HMI_SUB_URL)
            self.hmi_sub.setsockopt_string(zmq.SUBSCRIBE, HMI_TOPIC)

            self.steer_sub = self.context.socket(zmq.SUB)
            self.steer_sub.connect(ZMQ_STEER_SUB_URL)
            self.steer_sub.setsockopt_string(zmq.SUBSCRIBE, LANE_ASSIST_TOPIC)
            
            # --- PERBAIKAN DI SINI ---
            # Subscriber (penerima) harus selalu connect()
            self.distance_sub = self.context.socket(zmq.SUB)
            self.distance_sub.connect(ZMQ_DISTANCE_SUB_URL) 
            self.distance_sub.setsockopt_string(zmq.SUBSCRIBE, DISTANCE_TOPIC)
            # --------------------------

            self.llc_pub = self.context.socket(zmq.PUB)
            self.llc_pub.bind(ZMQ_LLC_PUB_URL)

            self.control_poller.register(self.hmi_sub, zmq.POLLIN)
            self.control_poller.register(self.steer_sub, zmq.POLLIN)
            self.control_poller.register(self.distance_sub, zmq.POLLIN)
            return True
        except zmq.ZMQError as e:
            self.logger.error(f"ZMQ Error: {e}")
            return False

    # (Sisa dari kelas tidak ada yang berubah, gunakan versi lengkap dari respons sebelumnya)
    def on_activate(self) -> bool:
        self.logger.info("Activating Control Node...")
        self.active_event.set()
        self.processing_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.processing_thread.start()
        return True

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating Control Node...")
        self.active_event.clear()
        if self.processing_thread: self.processing_thread.join(timeout=1.0)
        self._send_llc_command(0.0, 0.0, 100)
        return True

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down Control Node...")
        if self.state == NodeState.ACTIVE: self.on_deactivate()
        if self.hmi_sub: self.hmi_sub.close()
        if self.steer_sub: self.steer_sub.close()
        if self.distance_sub: self.distance_sub.close()
        if self.llc_pub: self.llc_pub.close()
        return True

    def _control_loop(self):
        self.logger.info("Control loop started.")
        
        while self.active_event.is_set():
            socks = dict(self.control_poller.poll(100))

            if self.hmi_sub in socks:
                topic, msg = self.hmi_sub.recv_multipart()
                command = msg.decode('utf-8')
                if command == "START" and not self.is_running:
                    self.is_running = True
                    self.time_stopped = None
                    self.time_started = time.time() # Add this line
                    self.logger.info("START command received. Vehicle moving.")
                elif command == "STOP" and self.is_running:
                    self.is_running = False
                    self.time_stopped = time.time()
                    self.time_started = None # Add this line
                    self.logger.info("STOP command received. Vehicle stopping.")
                elif "REV" in command:
                    rev_state = bool(int(command[3]))
                    self.logger.info(f"Reverse is {'on' if rev_state else 'off'} from HMI.")
                    self.is_reverse = rev_state

            current_time = time.time()
            if (current_time - self.last_log_time) >= self.log_interval:
                if self.last_received_distance is not None:
                    distance = self.last_received_distance
                    log_message = f"DATA: Jarak terakhir {distance:.2f} m. "
                    if distance < DISTANCE_THRESHOLD:
                        # log_message += "KEPUTUSAN: Peringatan, jarak terlalu dekat!"
                        self.logger.warning(log_message)
                    else:
                        # log_message += "KEPUTUSAN: Aman."
                        self.logger.info(log_message)
                    self.last_received_distance = None
                else:
                    self.logger.info("STATUS: Tidak ada data jarak baru yang masuk.")
                self.last_log_time = current_time
            if self.distance_sub in socks:
                topic, msg = self.distance_sub.recv_multipart()
                self.last_received_distance = float(msg.decode('utf-8'))

            if self.last_received_distance is not None and self.last_received_distance < DISTANCE_THRESHOLD and self.is_running:
                self.is_running = False
                self.time_stopped = time.time()
                self.time_started = None
                self.logger.info(self.is_running)

            if self.steer_sub in socks:
                topic, serialized_data = self.steer_sub.recv_multipart()
                command = steering_command_pb2.SteeringCommand()
                command.ParseFromString(serialized_data)
                self.logger.info(f"Received steering command: {command.auto_steer_angle} degrees")
                if not self.is_reverse:
                    self.current_steer_angle = command.auto_steer_angle * -3.0
                #TODO: Keep the steering angle adjustment in llc_interface (* -3.0)
                #TODO: Stream camera overlay 

            #TODO: Handle reverse steering and speed by subscribing to the reverse topic

            brake_force = 0
            if self.is_running:
                if self.time_started is not None:
                    elapsed_time = time.time() - self.time_started
                    self.current_speed_rpm = min(MAX_SPEED_RPM, elapsed_time * SPEED_RAMP_RATE)
                    if self.is_reverse:
                        self.current_speed_rpm *= -1
                    # TODO: Use v max from linefollowing_rs
                    # TODO: Handle negative speed for reverse in llc_interface
            else:
                self.current_speed_rpm = 0
                if self.time_stopped is not None:
                    elapsed_time = time.time() - self.time_stopped
                    brake_force = min(MAX_BRAKE_FORCE, int(elapsed_time * BRAKE_RAMP_RATE))
            # self._send_llc_command(0, self.current_steer_angle, 0)

            self._send_llc_command(self.current_speed_rpm, self.current_steer_angle, brake_force)
            time.sleep(0.02)
        self.logger.info("Control loop stopped.")
    
    def _send_llc_command(self, speed, steer, brake):
        command = { "speed_rpm": speed, "steer_angle": steer, "brake_force": brake }
        self.llc_pub.send_string(LLC_TOPIC, flags=zmq.SNDMORE)
        self.llc_pub.send_json(command)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    control_node = ControlNode()
    control_node.run()