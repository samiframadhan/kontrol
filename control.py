import zmq
import time
import json
import logging
import threading
from managednode import ManagedNode
import steering_command_pb2

# Configuration
ZMQ_HMI_SUB_URL = "ipc:///tmp/hmi_commands.ipc"
HMI_TOPIC = "hmi_cmd"
ZMQ_STEER_SUB_URL = "ipc:///tmp/teleop_cmd.ipc"
LANE_ASSIST_TOPIC = "lane_assist_angle"
ZMQ_LLC_PUB_URL = "ipc:///tmp/llc.ipc"
LLC_TOPIC = "teleop_cmd"

MAX_SPEED_RPM = 1000.0
SPEED_RAMP_RATE = 500  # RPM increase per second
BRAKE_RAMP_RATE = 50  # Brake force percentage increase per second
MAX_BRAKE_FORCE = 100 # Maximum brake force percentage

class ControlNode(ManagedNode):
    def __init__(self, node_name="control_node"):
        super().__init__(node_name)
        self.hmi_sub = None
        self.steer_sub = None
        self.llc_pub = None
        self.control_poller = zmq.Poller()

        self.is_running = False
        self.current_speed_rpm = 0.0
        self.current_steer_angle = 0.0
        self.time_stopped = None
        self.time_started = None

        self.processing_thread = None
        self.active_event = threading.Event()

    def on_configure(self) -> bool:
        self.logger.info("Configuring Control Node...")
        try:
            # self.hmi_sub = self.context.socket(zmq.SUB)
            # self.hmi_sub.connect(ZMQ_HMI_SUB_URL)
            # self.hmi_sub.setsockopt_string(zmq.SUBSCRIBE, HMI_TOPIC)

            self.steer_sub = self.context.socket(zmq.SUB)
            self.steer_sub.connect(ZMQ_STEER_SUB_URL)
            self.steer_sub.setsockopt_string(zmq.SUBSCRIBE, LANE_ASSIST_TOPIC)

            self.llc_pub = self.context.socket(zmq.PUB)
            self.llc_pub.bind(ZMQ_LLC_PUB_URL)

            # self.control_poller.register(self.hmi_sub, zmq.POLLIN)
            self.control_poller.register(self.steer_sub, zmq.POLLIN)
            return True
        except zmq.ZMQError as e:
            self.logger.error(f"ZMQ Error: {e}")
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
        # Send a final stop command
        self._send_llc_command(0.0, 0.0, 100)
        return True

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down Control Node...")
        if self.state == "active":
            self.on_deactivate()
        # if self.hmi_sub: self.hmi_sub.close()
        if self.steer_sub: self.steer_sub.close()
        if self.llc_pub: self.llc_pub.close()
        return True

    def _control_loop(self):
        self.logger.info("Control loop started.")
        self.is_running = True
        self.time_stopped = None
        self.time_started = time.time() # Add this line
        self.logger.info("START command received. Vehicle moving.")
        while self.active_event.is_set():
            socks = dict(self.control_poller.poll(100))

            # if self.hmi_sub in socks:
            #     topic, msg = self.hmi_sub.recv_multipart()
            #     command = msg.decode('utf-8')
                # self.is_running = True
                # self.time_stopped = None
                # self.time_started = time.time() # Add this line
                # self.logger.info("START command received. Vehicle moving.")
                # if command == "START" and not self.is_running:
                #     self.is_running = True
                #     self.time_stopped = None
                #     self.time_started = time.time() # Add this line
                #     self.logger.info("START command received. Vehicle moving.")
                # elif command == "STOP" and self.is_running:
                #     self.is_running = False
                #     self.time_stopped = time.time()
                #     self.time_started = None # Add this line
                #     self.logger.info("STOP command received. Vehicle stopping.")

            if self.steer_sub in socks:
                topic, serialized_data = self.steer_sub.recv_multipart()
                logging.info(f"Received steering command on topic '{topic.decode('utf-8')}'")
                command = steering_command_pb2.SteeringCommand()
                command.ParseFromString(serialized_data)
                self.logger.info(f"Received steering command: {command.auto_steer_angle} degrees")
                self.current_steer_angle = command.auto_steer_angle * -3.0

            brake_force = 0
            if self.is_running:
                if self.time_started is not None:
                    elapsed_time = time.time() - self.time_started
                    self.current_speed_rpm = min(MAX_SPEED_RPM, elapsed_time * SPEED_RAMP_RATE)
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
        command = {
            "speed_rpm": speed,
            "steer_angle": steer,
            "brake_force": brake
        }
        self.llc_pub.send_string(LLC_TOPIC, flags=zmq.SNDMORE)
        self.llc_pub.send_json(command)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    control_node = ControlNode()
    control_node.run()