import serial
import zmq
import time
import logging
import threading
from managednode import ManagedNode

# Configuration
COM_PORT = '/dev/ttyTHS1'  # Example for Jetson Nano, change as needed
BAUD_RATE = 9600
ZMQ_PUB_URL = "ipc:///tmp/hmi_commands.ipc"
HMI_TOPIC = "hmi_cmd"

# TODO: Send data to HMI from llc_interface; Safety state, speed, steering angle, etc.

class HMINode(ManagedNode):
    def __init__(self, node_name="hmi_node"):
        super().__init__(node_name)
        self.ser = None
        self.pub_socket = None
        self.processing_thread = None
        self.active_event = threading.Event()

    def on_configure(self) -> bool:
        self.logger.info("Configuring HMI Node...")
        try:
            self.ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
            self.logger.info(f"Successfully opened serial port {COM_PORT}")
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(ZMQ_PUB_URL)
            self.logger.info(f"Publisher bound to {ZMQ_PUB_URL}")
            return True
        except serial.SerialException as e:
            self.logger.error(f"Could not open serial port '{COM_PORT}'. {e}")
            return False
        except zmq.ZMQError as e:
            self.logger.error(f"ZMQ Error: {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating HMI Node...")
        self.active_event.set()
        self.processing_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.processing_thread.start()
        return True

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating HMI Node...")
        self.active_event.clear()
        if self.processing_thread:
            self.processing_thread.join(timeout=1.0)
        return True

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down HMI Node...")
        if self.state == "active":
            self.on_deactivate()
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.logger.info("Serial port closed.")
        if self.pub_socket:
            self.pub_socket.close()
        return True

    def _read_serial_loop(self):
        self.logger.info("Listening for HMI commands on serial port...")
        while self.active_event.is_set():
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if "START" in line:
                        self.logger.info("Received START command from HMI.")
                        self.pub_socket.send_string(HMI_TOPIC, flags=zmq.SNDMORE)
                        self.pub_socket.send_string("START")
                    elif "STOP" in line:
                        self.logger.info("Received STOP command from HMI.")
                        self.pub_socket.send_string(HMI_TOPIC, flags=zmq.SNDMORE)
                        self.pub_socket.send_string("STOP")
                except UnicodeDecodeError:
                    self.logger.warning("Received non-UTF-8 characters from serial port.")
            time.sleep(0.1)
        self.logger.info("HMI serial loop stopped.")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    hmi_node = HMINode()
    hmi_node.run()