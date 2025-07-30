# llc_interface.py (Modified for Batch Command)
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

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'  # Change this to your port
BAUDRATE = 115200
HEADER = 0xA5

# --- ZMQ Configuration ---
ZMQ_SUB_URL = "ipc:///tmp/llc.ipc" # Receives commands from teleop
ZMQ_PUB_URL = "tcp://*:5556"         # Publishes sensor data

# --- Function Codes ---
FUNC_STOP_STREAM = 0x00
FUNC_START_STREAM = 0x01
FUNC_CONTROL = 0x03  # This is the Batch Command
FUNC_INCOMING_DATA = 0x11
DOWNSAMPLE_RATE = 10  # Hz, the rate commands will be sent to the vehicle

# --- Control Parameters ---
BRAKE_RAMP_RATE = 50  # Brake force percentage increase per second
MAX_BRAKE_FORCE = 100 # Maximum brake force percentage

# --- Thread-safe object to store the latest command ---
class LatestCommand:
    """A thread-safe class to store the latest command."""
    def __init__(self):
        self.lock = threading.Lock()
        self.speed_rpm = 0.0
        self.steer_angle = 0.0
        self.brake_force = 0 # New
        self.last_command_time = time.time()

    def set_command(self, command):
        with self.lock:
            self.speed_rpm = command.get("speed_rpm", self.speed_rpm)
            self.steer_angle = command.get("steer_angle", self.steer_angle)
            self.brake_force = command.get("brake_force", self.brake_force) # New
            self.last_command_time = time.time()

            # Clamp values to be safe
            self.speed_rpm = max(-1500.0, min(1500.0, self.speed_rpm))
            self.steer_angle = max(-120.0, min(120.0, self.steer_angle))
            self.brake_force = max(0, min(100, self.brake_force)) # New

    def get_command(self):
        with self.lock:
            if time.time() - self.last_command_time > 0.5: # 500ms timeout
                self.speed_rpm = 0.0
                self.brake_force = 100 # Emergency brake
            return self.speed_rpm, self.steer_angle, self.brake_force

# --- MODIFIED Packet Creation ---
def create_packet(func_code, payload=b''):
    """
    Creates a complete serial packet.
    Args:
        func_code (int): The function code for the command.
        payload (bytes): The pre-packed data payload.
    """
    packet_without_checksum = bytearray([HEADER, func_code]) + payload
    checksum = sum(packet_without_checksum) & 0xFF
    return packet_without_checksum + bytearray([checksum])

def parse_stream_data(bytes_received, logger):
    if len(bytes_received) != 32:
        logger.warning(f"Incorrect packet length. Expected 32, got {len(bytes_received)}.")
        return None
    full_packet_for_checksum = bytearray([HEADER, FUNC_INCOMING_DATA]) + bytes_received[:-1]
    checksum_calculated = sum(full_packet_for_checksum) & 0xFF
    if checksum_calculated != bytes_received[-1]:
        logger.warning(f"Checksum mismatch! Calculated: {hex(checksum_calculated)}, Received: {hex(bytes_received[-1])}")
        return None
    data = bytes_received[:-1]
    format_string = '>BcHBbhhhhhhBBBccff'
    if len(data) != struct.calcsize(format_string):
        logger.error(f"Data length ({len(data)}) does not match format string size.")
        return None
    try:
        parsed_data = struct.unpack(format_string, data)
        return {
            "battery_%": parsed_data[0], "charging": 'Active' if parsed_data[1] else 'Inactive',
            "rpm": parsed_data[2] / 10.0, "brake_%": parsed_data[3], "steer_angle": parsed_data[4],
            "imu_x": parsed_data[5], "imu_y": parsed_data[6], "imu_z": parsed_data[7],
            "imu_yaw": parsed_data[8], "imu_pitch": parsed_data[9], "imu_roll": parsed_data[10],
            "radar_m": parsed_data[11] / 100.0, "ultrasonic_front_m": parsed_data[12] / 100.0,
            "ultrasonic_rear_m": parsed_data[13] / 100.0, "bumper_front": 'Active' if parsed_data[14] else 'Inactive',
            "bumper_rear": 'Active' if parsed_data[15] else 'Inactive', "gps_lat": parsed_data[16],
            "gps_lon": parsed_data[17]
        }
    except struct.error as e:
        logger.error(f"Failed to unpack data: {e}")
        return None


# --- Unchanged serial_io_thread, command_subscriber, data_publisher ---
def serial_io_thread(ser, read_queue, send_queue, shutdown_event, logger):
    logger.info("Serial I/O thread started.")
    while not shutdown_event.is_set():
        try:
            if ser.in_waiting > 0:
                if ser.read(1) == bytes([HEADER]):
                    f_code = ser.read(1)
                    if f_code == bytes([FUNC_INCOMING_DATA]):
                        packet_data = ser.read(32) # Data + Checksum
                        if len(packet_data) == 32:
                            parsed_info = parse_stream_data(packet_data, logger=logger)
                            if parsed_info:
                                read_queue.put(parsed_info)
            try:
                packet_to_send = send_queue.get_nowait()
                ser.write(packet_to_send)
                # logger.debug(f"Sent packet: {packet_to_send.hex(' ')}")
                send_queue.task_done()
            except Empty:
                pass
            time.sleep(0.005)
        except (serial.SerialException, IOError) as e:
            logger.error(f"Serial error in I/O thread: {e}", exc_info=True)
            shutdown_event.set()
            break
    logger.info("Serial I/O thread finished.")

def command_subscriber(latest_command, context, shutdown_event, logger):
    socket = context.socket(zmq.SUB)
    socket.connect(ZMQ_SUB_URL)
    socket.setsockopt_string(zmq.SUBSCRIBE, "teleop_cmd")
    logger.info(f"Listening for commands on {ZMQ_SUB_URL}")
    while not shutdown_event.is_set():
        try:
            if socket.poll(100):
                topic, command_json = socket.recv_multipart()
                command = json.loads(command_json)
                latest_command.set_command(command)
                logger.info(f"Received command: {command}")
        except (zmq.ZMQError, json.JSONDecodeError) as e:
            logger.error(f"Error in command subscriber: {e}", exc_info=True)
            shutdown_event.set()
            break
    socket.close()
    logger.info("Command subscriber thread finished.")

# --- REWRITTEN control_sender_thread ---
def control_sender_thread(latest_command, send_queue, shutdown_event, logger):
    """
    This thread constructs and sends a single Batch Command packet at a fixed rate.
    """
    logger.info(f"Control sender started. Sending Batch Commands at {DOWNSAMPLE_RATE} Hz.")

    while not shutdown_event.is_set():
        start_time = time.time()
        try:
            # 1. Get the latest high-level command
            speed_rpm, steer_angle, brake_force = latest_command.get_command()

            # 2. Translate into low-level components for the packet
            # RPM and Direction
            direction = 0  # 0 for forward
            rpm_to_send = int(abs(speed_rpm))
            if speed_rpm < 0:
                direction = 1  # 1 for reverse

            # Steer angle
            steer_to_send = int(steer_angle)

            # Brake force is now directly commanded
            brake_to_send = int(brake_force)

            # 3. Pack the data into a 5-byte payload
            # Format: RPM (uint16), Direction (uint8), Brake (uint8), Steer (int8)
            # Endianness: Big-endian (>)
            payload = struct.pack('>HBBb', rpm_to_send, direction, brake_to_send, steer_to_send)

            # 4. Create the final packet with header, function code, and checksum
            packet = create_packet(FUNC_CONTROL, payload)
            logger.debug(f"Created packet: {packet.hex(' ')}")

            # 5. Send the packet
            send_queue.put(packet)

            logger.info(f"CMD > RPM: {speed_rpm:<5.0f}, Steer: {steer_to_send:<4}, Brake: {brake_to_send:<3}% | Packet: {packet.hex(' ')}")

        except Exception as e:
            logger.error(f"Error in control sender: {e}", exc_info=True)
            shutdown_event.set()
            break

        # 6. Maintain the desired send rate
        time_to_sleep = (1.0 / DOWNSAMPLE_RATE) - (time.time() - start_time)
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)

    logger.info("Control sender thread finished.")


def data_publisher(read_queue, context, shutdown_event, logger):
    socket = context.socket(zmq.PUB)
    socket.bind(ZMQ_PUB_URL)
    logger.info(f"Publishing sensor data on {ZMQ_PUB_URL}")
    while not shutdown_event.is_set():
        try:
            parsed_info = read_queue.get(timeout=1)
            socket.send_string("sensor_data", flags=zmq.SNDMORE)
            socket.send_json(parsed_info)
            read_queue.task_done()
        except Empty:
            continue
        except Exception as e:
            logger.error(f"Error in data publisher: {e}", exc_info=True)
            shutdown_event.set()
            break
    socket.close()
    logger.info("Data publisher thread finished.")


# --- Main Managed Node Class (Unchanged from original structure) ---
class LLCInterfaceNode(ManagedNode):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.ser = None
        self.read_queue = Queue()
        self.send_queue = Queue()
        self.latest_command = LatestCommand()
        self.threads = []
        self.pub_socket = None
        self.sub_socket = None

    def on_configure(self) -> bool:
        self.logger.info("Configuring LLC Interface Node...")
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            self.logger.info(f"Successfully opened serial port {SERIAL_PORT}")
            return True
        except serial.SerialException as e:
            self.logger.error(f"Could not open serial port '{SERIAL_PORT}'. {e}")
            return False

    def on_activate(self) -> bool:
        self.logger.info("Activating LLC Interface Node...")
        while not self.send_queue.empty(): self.send_queue.get_nowait()
        
        self.send_queue.put(create_packet(FUNC_STOP_STREAM))
        time.sleep(0.1)
        self.send_queue.put(create_packet(FUNC_START_STREAM))
        self.logger.info("Requested data stream start from vehicle.")

        io_thread = threading.Thread(target=serial_io_thread, args=(self.ser, self.read_queue, self.send_queue, self.shutdown_event, self.logger), name="SerialIOThread")
        sub_thread = threading.Thread(target=command_subscriber, args=(self.latest_command, self.context, self.shutdown_event, self.logger), name="CommandSubThread")
        pub_thread = threading.Thread(target=data_publisher, args=(self.read_queue, self.context, self.shutdown_event, self.logger), name="DataPubThread")
        sender_thread = threading.Thread(target=control_sender_thread, args=(self.latest_command, self.send_queue, self.shutdown_event, self.logger), name="ControlSenderThread")
        
        self.threads = [io_thread, sub_thread, pub_thread, sender_thread]
        for t in self.threads:
            t.start()
        
        self.logger.info("All threads started.")
        return True

    def on_deactivate(self) -> bool:
        self.logger.info("Deactivating LLC Interface Node...")
        self.shutdown_event.set()

        if self.ser and self.ser.is_open:
            try:
                self.logger.info("Stopping data stream and sending final stop command.")
                self.ser.write(create_packet(FUNC_STOP_STREAM))
                time.sleep(0.1)
                # Send one last batch command to stop everything
                payload = struct.pack('>HBBb', 0, 0, 100, 0) # 0 RPM, 0 DIR, 100% BRAKE, 0 STEER
                self.ser.write(create_packet(FUNC_CONTROL, payload))
                time.sleep(0.1)
            except serial.SerialException as e:
                self.logger.error(f"Error sending deactivation commands: {e}")

        for t in self.threads:
            t.join(timeout=1.5)
        self.logger.info("All threads joined.")
        
        for handler in logging.getLogger().handlers:
            handler.flush()
            handler.close()
        logging.getLogger().handlers.clear()
        
        self.shutdown_event.clear()
        return True

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down LLC Interface Node...")
        if self.ser and self.ser.is_open:
            try:
                self.logger.info("Applying full brake and centering steer on exit.")
                payload = struct.pack('>HBBb', 0, 0, 100, 0) # 0 RPM, 0 DIR, 100% BRAKE, 0 STEER
                self.ser.write(create_packet(FUNC_CONTROL, payload))
                time.sleep(0.2)
            except serial.SerialException as e:
                self.logger.error(f"Error sending final commands: {e}")
            finally:
                self.ser.close()
                self.logger.info("Serial port closed.")
        return True

if __name__ == "__main__":
    log_filename = f"llc_managed_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logging.basicConfig(
        level=logging.INFO, # Changed to INFO for less verbose default logging
        format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s',
        handlers=[
            logging.FileHandler(log_filename),
            logging.StreamHandler(sys.stdout)
        ]
    )
    
    llc_node = LLCInterfaceNode(node_name="llc_interface")
    llc_node.run()