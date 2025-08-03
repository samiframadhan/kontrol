# llc_interface_node.py (Updated for unified config)
import serial
import struct
import threading
import time
import sys
import zmq
import json
import logging
from datetime import datetime
from queue import Queue, Empty
from managednode import ManagedNode
from config_mixin import ConfigMixin

class LatestCommand:
    """A thread-safe class to store the latest command."""
    def __init__(self, steer_multiplier: float = -3.0):
        self.lock = threading.Lock()
        self.speed_rpm = 0.0
        self.steer_angle = 0.0
        self.brake_force = 0
        self.steer_multiplier = steer_multiplier
        self.last_command_time = time.time()

    def set_command(self, command):
        with self.lock:
            self.speed_rpm = command.get("speed_rpm", self.speed_rpm)
            self.steer_angle = command.get("steer_angle", self.steer_angle)
            self.brake_force = command.get("brake_force", self.brake_force)
            self.steer_angle = self.steer_angle * self.steer_multiplier
            self.last_command_time = time.time()

            # Clamp values to be safe
            self.speed_rpm = max(-1500.0, min(1500.0, self.speed_rpm))
            self.steer_angle = max(-120.0, min(120.0, self.steer_angle))
            self.brake_force = max(0, min(100, self.brake_force))

    def get_command(self):
        with self.lock:
            if time.time() - self.last_command_time > 0.5:  # 500ms timeout
                self.speed_rpm = 0.0
                self.brake_force = 100  # Emergency brake
            return self.speed_rpm, self.steer_angle, self.brake_force

class LLCInterfaceNode(ManagedNode, ConfigMixin):
    def __init__(self, node_name="llc_interface", config_path="config.yaml"):
        ManagedNode.__init__(self, node_name)
        ConfigMixin.__init__(self, config_path)
        
        # Get LLC configuration
        llc_config = self.get_section_config('llc')
        
        self.serial_port = llc_config['serial_port']
        self.baudrate = llc_config['baudrate']
        self.steer_multiplier = llc_config['steer_multiplier']
        self.downsample_rate = llc_config['downsample_rate']
        
        # Serial and communication
        self.ser = None
        self.read_queue = Queue()
        self.send_queue = Queue()
        self.latest_command = LatestCommand(self.steer_multiplier)
        self.threads = []
        
        # ZMQ Sockets
        self.control_sub = None
        self.sensor_pub = None
        self.control_poller = None
        
        # Protocol constants
        self.HEADER = 0xA5
        self.FUNC_STOP_STREAM = 0x00
        self.FUNC_START_STREAM = 0x01
        self.FUNC_CONTROL = 0x03
        self.FUNC_INCOMING_DATA = 0x11

    def on_configure(self) -> bool:
        self.logger.info("Configuring LLC Interface Node...")
        try:
            # Configure serial port
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.logger.info(f"Successfully opened serial port {self.serial_port}")

            # --- Configure ZMQ Sockets ---
            self.control_sub = self.context.socket(zmq.SUB)
            self.control_sub.connect(self.get_zmq_url('control_cmd_url'))
            self.control_sub.setsockopt_string(zmq.SUBSCRIBE, self.get_zmq_topic('control_cmd_topic'))
            self.logger.info(f"Control subscriber connected to {self.get_zmq_url('control_cmd_url')}")
            
            self.sensor_pub = self.context.socket(zmq.PUB)
            self.sensor_pub.bind(self.get_zmq_url('sensor_data_url'))
            self.logger.info(f"Sensor publisher bound to {self.get_zmq_url('sensor_data_url')}")

            # Poller for control commands
            self.control_poller = zmq.Poller()
            self.control_poller.register(self.control_sub, zmq.POLLIN)

            return True
        except (serial.SerialException, zmq.ZMQError) as e:
            self.logger.error(f"Configuration failed: {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating LLC Interface Node...")
        
        try:
            # Clear any pending commands
            while not self.send_queue.empty():
                self.send_queue.get_nowait()
            
            # Start/stop stream commands
            self.send_queue.put(self._create_packet(self.FUNC_STOP_STREAM))
            time.sleep(0.1)
            self.send_queue.put(self._create_packet(self.FUNC_START_STREAM))
            self.logger.info("Requested data stream start from vehicle.")

            # Start all threads
            io_thread = threading.Thread(
                target=self._serial_io_thread, 
                name="SerialIOThread"
            )
            sub_thread = threading.Thread(
                target=self._command_subscriber, 
                name="CommandSubThread"
            )
            pub_thread = threading.Thread(
                target=self._data_publisher, 
                name="DataPubThread"
            )
            sender_thread = threading.Thread(
                target=self._control_sender_thread, 
                name="ControlSenderThread"
            )
            
            self.threads = [io_thread, sub_thread, pub_thread, sender_thread]
            for t in self.threads:
                t.start()
            
            self.logger.info("All LLC Interface threads started.")
            return True
        except Exception as e:
            self.logger.error(f"Activation failed: {e}")
            return False

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating LLC Interface Node...")
        try:
            self.shutdown_event.set()

            if self.ser and self.ser.is_open:
                try:
                    self.logger.info("Stopping data stream and sending final stop command.")
                    self.ser.write(self._create_packet(self.FUNC_STOP_STREAM))
                    time.sleep(0.1)
                    # Send one last batch command to stop everything
                    payload = struct.pack('>HBBb', 0, 0, 100, 0)  # 0 RPM, 0 DIR, 100% BRAKE, 0 STEER
                    self.ser.write(self._create_packet(self.FUNC_CONTROL, payload))
                    time.sleep(0.1)
                except serial.SerialException as e:
                    self.logger.error(f"Error sending deactivation commands: {e}")

            for t in self.threads:
                t.join(timeout=1.5)
            self.logger.info("All LLC Interface threads joined.")
            
            self.shutdown_event.clear()
            return True
        except Exception as e:
            self.logger.error(f"Deactivation failed: {e}")
            return False

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down LLC Interface Node...")
        try:
            # Ensure threads are stopped if node is directly shut down
            if not self.shutdown_event.is_set():
                self.on_deactivate()

            if self.ser and self.ser.is_open:
                self.logger.info("Applying full brake and centering steer on exit.")
                payload = struct.pack('>HBBb', 0, 0, 100, 0)  # 0 RPM, 0 DIR, 100% BRAKE, 0 STEER
                self.ser.write(self._create_packet(self.FUNC_CONTROL, payload))
                time.sleep(0.2)
        except serial.SerialException as e:
            self.logger.error(f"Error sending final commands: {e}")
        finally:
            if self.ser: self.ser.close()
            if self.control_sub: self.control_sub.close()
            if self.sensor_pub: self.sensor_pub.close()
            self.logger.info("Serial port and ZMQ sockets closed.")
        return True

    def _create_packet(self, func_code, payload=b''):
        """Creates a complete serial packet."""
        packet_without_checksum = bytearray([self.HEADER, func_code]) + payload
        checksum = sum(packet_without_checksum) & 0xFF
        return packet_without_checksum + bytearray([checksum])

    def _parse_stream_data(self, bytes_received):
        """Parse incoming sensor data from vehicle."""
        if len(bytes_received) != 32:
            self.logger.warning(f"Incorrect packet length. Expected 32, got {len(bytes_received)}.")
            return None
            
        full_packet_for_checksum = bytearray([self.HEADER, self.FUNC_INCOMING_DATA]) + bytes_received[:-1]
        checksum_calculated = sum(full_packet_for_checksum) & 0xFF
        
        if checksum_calculated != bytes_received[-1]:
            self.logger.warning(f"Checksum mismatch! Calculated: {hex(checksum_calculated)}, Received: {hex(bytes_received[-1])}")
            return None
            
        data = bytes_received[:-1]
        format_string = '>BcHBbhhhhhhBBBccff'
        
        if len(data) != struct.calcsize(format_string):
            self.logger.error(f"Data length ({len(data)}) does not match format string size.")
            return None
            
        try:
            parsed_data = struct.unpack(format_string, data)
            return {
                "battery_%": parsed_data[0], 
                "charging": 'Active' if parsed_data[1] else 'Inactive',
                "rpm": parsed_data[2] / 10.0, 
                "brake_%": parsed_data[3], 
                "steer_angle": parsed_data[4],
                "imu_x": parsed_data[5], 
                "imu_y": parsed_data[6], 
                "imu_z": parsed_data[7],
                "imu_yaw": parsed_data[8], 
                "imu_pitch": parsed_data[9], 
                "imu_roll": parsed_data[10],
                "radar_m": parsed_data[11] / 100.0, 
                "ultrasonic_front_m": parsed_data[12] / 100.0,
                "ultrasonic_rear_m": parsed_data[13] / 100.0, 
                "bumper_front": 'Active' if parsed_data[14] else 'Inactive',
                "bumper_rear": 'Active' if parsed_data[15] else 'Inactive', 
                "gps_lat": parsed_data[16],
                "gps_lon": parsed_data[17]
            }
        except struct.error as e:
            self.logger.error(f"Failed to unpack data: {e}")
            return None

    def _serial_io_thread(self):
        """Handle serial I/O operations."""
        self.logger.info("Serial I/O thread started.")
        while not self.shutdown_event.is_set():
            try:
                if self.ser.in_waiting > 0:
                    if self.ser.read(1) == bytes([self.HEADER]):
                        f_code = self.ser.read(1)
                        if f_code == bytes([self.FUNC_INCOMING_DATA]):
                            packet_data = self.ser.read(32)  # Data + Checksum
                            if len(packet_data) == 32:
                                parsed_info = self._parse_stream_data(packet_data)
                                if parsed_info:
                                    self.read_queue.put(parsed_info)
                try:
                    packet_to_send = self.send_queue.get_nowait()
                    self.ser.write(packet_to_send)
                    self.send_queue.task_done()
                except Empty:
                    pass
                time.sleep(0.005)
            except (serial.SerialException, IOError) as e:
                self.logger.error(f"Serial error in I/O thread: {e}", exc_info=True)
                self.shutdown_event.set()
                break
        self.logger.info("Serial I/O thread finished.")

    def _command_subscriber(self):
        """Subscribe to control commands using the pre-configured socket."""
        self.logger.info("Command subscriber thread started.")
        
        while not self.shutdown_event.is_set():
            try:
                if self.control_sub.poll(100):
                    topic, command_json = self.control_sub.recv_multipart()
                    command = json.loads(command_json)
                    self.latest_command.set_command(command)
                    self.logger.info(f"Received command: {command}")
            except (zmq.ZMQError, json.JSONDecodeError) as e:
                if not self.shutdown_event.is_set():
                    self.logger.error(f"Error in command subscriber: {e}", exc_info=True)
                self.shutdown_event.set()
                break
        self.logger.info("Command subscriber thread finished.")

    def _data_publisher(self):
        """Publish sensor data using the pre-configured socket."""
        self.logger.info("Data publisher thread started.")
        
        while not self.shutdown_event.is_set():
            try:
                parsed_info = self.read_queue.get(timeout=1)
                self.sensor_pub.send_string(self.get_zmq_topic('sensor_data_topic'), flags=zmq.SNDMORE)
                self.sensor_pub.send_json(parsed_info)
                self.read_queue.task_done()
            except Empty:
                continue
            except Exception as e:
                if not self.shutdown_event.is_set():
                    self.logger.error(f"Error in data publisher: {e}", exc_info=True)
                self.shutdown_event.set()
                break
        self.logger.info("Data publisher thread finished.")

    def _control_sender_thread(self):
        """Send control commands to vehicle at fixed rate."""
        self.logger.info(f"Control sender started. Sending Batch Commands at {self.downsample_rate} Hz.")

        while not self.shutdown_event.is_set():
            start_time = time.time()
            try:
                # Get the latest high-level command
                speed_rpm, steer_angle, brake_force = self.latest_command.get_command()

                # Translate into low-level components for the packet
                direction = 0 if speed_rpm >= 0 else 1
                rpm_to_send = int(abs(speed_rpm))
                steer_to_send = int(steer_angle)
                brake_to_send = int(brake_force)

                # Pack the data into a 5-byte payload
                payload = struct.pack('>HBBb', rpm_to_send, direction, brake_to_send, steer_to_send)

                # Create the final packet
                packet = self._create_packet(self.FUNC_CONTROL, payload)
                self.send_queue.put(packet)

                self.logger.info(f"CMD > RPM: {speed_rpm:<5.0f}, Steer: {steer_to_send:<4}, Brake: {brake_to_send:<3}%")

            except Exception as e:
                if not self.shutdown_event.is_set():
                    self.logger.error(f"Error in control sender: {e}", exc_info=True)
                self.shutdown_event.set()
                break

            # Maintain the desired send rate
            time_to_sleep = (1.0 / self.downsample_rate) - (time.time() - start_time)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

        self.logger.info("Control sender thread finished.")

if __name__ == "__main__":
    log_filename = f"llc_managed_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logging.basicConfig(
        level=logging.DEBUG, # Change to DEBUG to see command and data logs
        format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s',
        handlers=[
            logging.FileHandler(log_filename),
            logging.StreamHandler(sys.stdout)
        ]
    )
    
    llc_node = LLCInterfaceNode()
    llc_node.run()