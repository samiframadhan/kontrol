import serial
import zmq
import time
import logging
import threading
import json
from queue import Queue, Empty
from managednode import ManagedNode
from config_mixin import ConfigMixin

class HMINode(ManagedNode, ConfigMixin):
    def __init__(self, node_name="hmi_node", config_path="config.yaml"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        self.ser = None
        self.cmd_pub = None
        self.sensor_sub = None

        self.threads = []
        self.send_queue = Queue()
        self.read_queue = Queue()

        self.is_reverse = False

    def on_configure(self) -> bool:
        self.logger.info("Configuring HMI Node...")
        try:
            hmi_config = self.get_section_config('hmi')
            
            # Configure Serial Port
            self.ser = serial.Serial(hmi_config['serial_port'], hmi_config['baud_rate'], timeout=1)
            self.logger.info(f"Successfully opened serial port {hmi_config['serial_port']}")
            
            # Configure ZMQ Publisher for outgoing HMI commands (START/STOP/DIRECTION)
            self.cmd_pub = self.context.socket(zmq.PUB)
            self.cmd_pub.bind(self.get_zmq_url('hmi_cmd_url'))
            self.logger.info(f"Command publisher bound to {self.get_zmq_url('hmi_cmd_url')}")

            # Configure ZMQ Subscriber for incoming LLC sensor data
            self.sensor_sub = self.context.socket(zmq.SUB)
            self.sensor_sub.connect(self.get_zmq_url('sensor_data_url'))
            self.sensor_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('sensor_data_topic'))
            self.logger.info(f"Sensor subscriber connected to {self.get_zmq_url('sensor_data_url')}")
            
            return True
        except (serial.SerialException, zmq.ZMQError, Exception) as e:
            self.logger.error(f"Configuration failed: {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating HMI Node...")
        try:
            # Clear queues before starting
            while not self.send_queue.empty():
                self.send_queue.get_nowait()
            while not self.read_queue.empty():
                self.read_queue.get_nowait()

            # Start all threads
            io_thread = threading.Thread(target=self._serial_io_thread, name="SerialIOThread")
            sensor_thread = threading.Thread(target=self._sensor_subscriber_thread, name="SensorSubThread")
            command_thread = threading.Thread(target=self._command_processor_thread, name="CommandProcThread")
            
            self.threads = [io_thread, sensor_thread, command_thread]
            for t in self.threads:
                t.start()
            
            self.logger.info("All HMI Node threads started.")
            return True
        except Exception as e:
            self.logger.error(f"Activation failed: {e}")
            return False

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating HMI Node...")
        try:
            self.shutdown_event.set()
            for t in self.threads:
                t.join(timeout=1.5)
            self.logger.info("All HMI Node threads joined.")
            self.shutdown_event.clear()
            return True
        except Exception as e:
            self.logger.error(f"Deactivation failed: {e}")
            return False

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down HMI Node...")
        try:
            if not self.shutdown_event.is_set():
                self.on_deactivate()
            if self.ser and self.ser.is_open:
                self.ser.close()
                self.logger.info("Serial port closed.")
            if self.cmd_pub: self.cmd_pub.close()
            if self.sensor_sub: self.sensor_sub.close()
            self.logger.info("ZMQ sockets closed.")
            return True
        except Exception as e:
            self.logger.error(f"Error during shutdown: {e}")
            return False

    def _send_to_hmi(self, command_string):
        """Helper function to format and queue a command for the HMI."""
        if not isinstance(command_string, str):
            return
        # The HMI requires commands to be terminated by three null bytes
        full_command = command_string.encode('ascii') + b'\xFF\xFF\xFF'
        self.send_queue.put(full_command)
        self.logger.debug(f"Queued for HMI: {command_string}")

    def _update_hmi_from_sensors(self, sensor_data):
        """Translate sensor data into HMI commands."""
        # --- States Page Updates ---
        # Battery percentage
        self._send_to_hmi(f"States.bat.val={int(sensor_data.get('battery_%', 0))}")
        
        # Steering angle
        steer_angle = int(sensor_data.get('steer_angle', 0))
        self._send_to_hmi(f"States.str.val={steer_angle}")
        
        # Speed in km/h (assuming RPM to km/h conversion)
        # NOTE: You might need to adjust this conversion factor
        rpm = sensor_data.get('rpm', 0)
        kmh = (rpm * 3.6 * 0.0025) # Example: 0.2m wheel circumference
        # self._send_to_hmi(f"States.spd.val={int(kmh)}")
        # self._send_to_hmi(f"Auto.curvel.val={int(kmh)}")


        # Steering gauge (percentage of -35 to 35)
        # Clamp between -35 and 35, then normalize to 0-100
        steer_percent = int(((max(-35, min(35, steer_angle)) + 35) / 70) * 100)
        self._send_to_hmi(f"States.strgage.val={steer_percent}")

        # Speed gauge (percentage of 0 to 30 km/h)
        spd_percent = int((max(0, min(30, kmh)) / 30) * 100)
        # self._send_to_hmi(f"States.spdgage.val={spd_percent}")

        # --- Indicator Pictures ---
        # Example: Show a warning if battery is below 20%
        #if sensor_data.get('battery_%', 100) < 20:
        #     self._send_to_hmi("States.batind.pco=12") # Show warning icon
        #else:
        #     self._send_to_hmi("States.batind.pco=13") # Hide warning icon

    def _process_incoming_hmi_byte(self, byte_received):
        """Process a single byte received from the HMI serial port."""
        if byte_received == b'\xA5':
            self.logger.info("Received START command from HMI.")
            self.cmd_pub.send_string(self.get_zmq_topic('hmi_cmd_topic'), flags=zmq.SNDMORE)
            self.cmd_pub.send_string("START")
            self.ser.read(2)  # Empty the next 5 bytes
            self.ser.flush()
        elif byte_received == b'\xA6':
            self.logger.info("Received STOP command from HMI.")
            self.cmd_pub.send_string(self.get_zmq_topic('hmi_cmd_topic'), flags=zmq.SNDMORE)
            self.cmd_pub.send_string("STOP")
            self.ser.read(2)  # Empty the next 5 bytes
            self.ser.flush()
        elif byte_received == b'\xA7':
            # This is a multi-byte command, read the next byte
            status = self.ser.read(1)
            direction = "reverse" if status == b'\x01' else "forward"
            self.logger.info(f"Received DIRECTION command from HMI: {direction}.")
            self.cmd_pub.send_string(self.get_zmq_topic('hmi_direction_topic'), flags=zmq.SNDMORE)
            self.cmd_pub.send_string(direction)
            self.ser.read(5)  # Empty the next 5 bytes
            self.ser.flush()
        elif byte_received == b'\xA8':
            self.logger.info("Received max speed command from HMI.")
            # Read the next byte for max speed (from hex \x00 - \xFF)
            max_speed_byte = self.ser.read(1)
            self.logger.info(f"Max speed byte received: {max_speed_byte.hex()}")
            # Empty the next 5 bytes
            self.ser.read(5)
            self.ser.flush()
            if max_speed_byte.hex() is not None:
                max_speed = int(max_speed_byte.hex())
                max_speed_rpm = int((max_speed / 3.6) / 0.0025)
                self.logger.info(f"Setting max speed to {max_speed_rpm} rpm.")
                self.cmd_pub.send_string(self.get_zmq_topic('hmi_max_speed_topic'), flags=zmq.SNDMORE)
                self.cmd_pub.send_string(str(max_speed_rpm))

    def _serial_io_thread(self):
        """Handle both reading from and writing to the serial port."""
        self.logger.info("Serial I/O thread started.")
        while not self.shutdown_event.is_set():
            try:
                # --- Reading from HMI ---
                if self.ser.in_waiting > 0:
                    byte_in = self.ser.read(1)
                    self._process_incoming_hmi_byte(byte_in)

                # --- Writing to HMI ---
                try:
                    command_to_send = self.send_queue.get_nowait()
                    self.ser.write(command_to_send)
                    self.send_queue.task_done()
                except Empty:
                    pass # It's normal for the send queue to be empty
            
            except (serial.SerialException, IOError) as e:
                self.logger.error(f"Serial error in I/O thread: {e}", exc_info=True)
                self.shutdown_event.set()
                break

            time.sleep(0.01) # Prevent high CPU usage
        self.logger.info("Serial I/O thread finished.")

    def _sensor_subscriber_thread(self):
        """Subscribe to LLC sensor data and put it in a queue."""
        self.logger.info("Sensor subscriber thread started.")
        while not self.shutdown_event.is_set():
            try:
                if self.sensor_sub.poll(100): # Poll with a timeout
                    topic, sensor_json = self.sensor_sub.recv_multipart()
                    sensor_data = json.loads(sensor_json)
                    self.read_queue.put(sensor_data)
            except (zmq.ZMQError, json.JSONDecodeError) as e:
                if not self.shutdown_event.is_set():
                    self.logger.error(f"Error in sensor subscriber: {e}", exc_info=True)
                break
        self.logger.info("Sensor subscriber thread finished.")

    def _command_processor_thread(self):
        """Process sensor data from the queue and generate HMI commands."""
        self.logger.info("Command processor thread started.")
        while not self.shutdown_event.is_set():
            try:
                # Get the latest sensor data from the queue
                sensor_data = self.read_queue.get(timeout=1)
                
                # Translate data and queue commands for the HMI
                self._update_hmi_from_sensors(sensor_data)
                
                self.read_queue.task_done()
            except Empty:
                continue # No new sensor data
            except Exception as e:
                if not self.shutdown_event.is_set():
                    self.logger.error(f"Error in command processor: {e}", exc_info=True)
                break
        self.logger.info("Command processor thread finished.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s')
    hmi_node = HMINode(config_path="../params/config.yaml")
    hmi_node.run()
