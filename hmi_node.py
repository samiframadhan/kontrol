import serial
import zmq
import time
import logging
import threading
from managednode import ManagedNode
from config_mixin import ConfigMixin


class HMINode(ManagedNode, ConfigMixin):
    def __init__(self, node_name="hmi_node", config_path="config.yaml"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        self.ser = None
        self.pub_socket = None
        self.processing_thread = None
        self.active_event = threading.Event()
        self.is_reverse = False

    def on_configure(self) -> bool:
        self.logger.info("Configuring HMI Node...")
        try:
            hmi_config = self.get_section_config('hmi')
            
            self.ser = serial.Serial(hmi_config['serial_port'], hmi_config['baud_rate'], timeout=1)
            self.logger.info(f"Successfully opened serial port {hmi_config['serial_port']}")
            
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(self.get_zmq_url('hmi_cmd_url'))
            self.logger.info(f"Publisher bound to {self.get_zmq_url('hmi_cmd_url')}")
            
            return True
        except Exception as e:
            self.logger.error(f"Configuration failed: {e}")
            return False

    def _read_serial_loop(self):
        self.logger.info("Listening for HMI commands on serial port...")
        while self.active_event.is_set():
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if "START" in line:
                        self.logger.info("Received START command from HMI.")
                        self.pub_socket.send_string(self.get_zmq_topic('hmi_cmd_topic'), flags=zmq.SNDMORE)
                        self.pub_socket.send_string("START")
                    elif "STOP" in line:
                        self.logger.info("Received STOP command from HMI.")
                        self.pub_socket.send_string(self.get_zmq_topic('hmi_cmd_topic'), flags=zmq.SNDMORE)
                        self.pub_socket.send_string("STOP")
                    elif "REV" in line:
                        if line[3].isdigit():
                            rev_state = int(line[3])
                            self.is_reverse = bool(rev_state)
                            direction = "reverse" if self.is_reverse else "forward"
                            self.logger.info(f"Direction set to {direction} from HMI.")
                            self.pub_socket.send_string(self.get_zmq_topic('hmi_direction_topic'), flags=zmq.SNDMORE)
                            self.pub_socket.send_string(direction)
                except UnicodeDecodeError:
                    self.logger.warning("Received non-UTF-8 characters from serial port.")
            time.sleep(0.1)
        self.logger.info("HMI serial loop stopped.")

    def on_activate(self) -> bool:
        self.logger.info("Activating HMI Node...")
        try:
            self.active_event.set()
            self.processing_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
            self.processing_thread.start()
            return True
        except Exception as e:
            self.logger.error(f"Activation failed: {e}")
            return False

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating HMI Node...")
        try:
            self.active_event.clear()
            if self.processing_thread:
                self.processing_thread.join(timeout=1.0)
            return True
        except Exception as e:
            self.logger.error(f"Deactivation failed: {e}")
            return False

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down HMI Node...")
        try:
            if self.state == "active":
                self.on_deactivate()
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.logger.info("Serial port closed.")
            if self.pub_socket:
                self.pub_socket.close()
            return True
        except Exception as e:
            self.logger.error(f"Error during shutdown: {e}")
            return False

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    hmi_node = HMINode()
    hmi_node.run()