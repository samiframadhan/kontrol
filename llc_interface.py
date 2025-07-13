# llc_interface.py
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

# Assuming managednode.py is in the same directory or accessible via PYTHONPATH
from managednode import ManagedNode, ORCHESTRATOR_URL

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'  # Change this to your port
BAUDRATE = 115200
HEADER = 0xA5

# --- ZMQ Configuration ---
# Note: The orchestrator URL is imported from managednode
ZMQ_SUB_URL = "ipc:///tmp/llc.ipc" # Receives commands from teleop
ZMQ_PUB_URL = "tcp://*:5556"         # Publishes sensor data

# --- Function Codes ---
FUNC_STOP_STREAM = 0x00
FUNC_START_STREAM = 0x01
FUNC_DRIVE = 0x03
FUNC_REVERSE = 0x04
FUNC_BRAKE = 0x05
FUNC_STEER = 0x06
FUNC_INCOMING_DATA = 0x11
DOWNSAMPLE_RATE = 10  # Hz, the rate commands will be sent to the vehicle

BRAKE_RAMP_RATE = 5  # Brake force percentage increase per second
MAX_BRAKE_FORCE = 20 # Maximum brake force percentage

# --- Thread-safe object to store the latest command (Unchanged from original) ---
class LatestCommand:
    """A thread-safe class to store the latest command."""
    def __init__(self):
        self.lock = threading.Lock()
        self.speed_rpm = 0.0
        self.steer_angle = 0.0
        self.time_stopped = None

    def set_command(self, command):
        with self.lock:
            new_speed_rpm = command.get("speed_rpm", self.speed_rpm)
            if new_speed_rpm == 0.0 and self.speed_rpm != 0.0:
                if self.time_stopped is None:
                    self.time_stopped = time.time()
            elif new_speed_rpm != 0.0:
                self.time_stopped = None
            
            new_steer_angle = command.get("steer_angle", self.steer_angle)
            if new_steer_angle == 0.0:
                new_steer_angle = self.steer_angle
            
            self.steer_angle = new_steer_angle
            self.speed_rpm = new_speed_rpm
            self.speed_rpm = max(-100.0, min(500.0, self.speed_rpm))
            self.steer_angle = max(-120.0, min(90.0, self.steer_angle))

    def get_command(self):
        with self.lock:
            return self.speed_rpm, self.steer_angle, self.time_stopped

# --- Packet Creation & Parsing (Unchanged from original) ---
def create_packet(func_code, payload_format=None, value=None):
    if payload_format and value is not None:
        payload = struct.pack(payload_format, value)
    else:
        payload = b''
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

# --- Thread Functions ---
# Note: shutdown_event is now passed from the ManagedNode instance
def serial_io_thread(ser, read_queue, send_queue, shutdown_event, logger):
    logger.info("Serial I/O thread started.")
    while not shutdown_event.is_set():
        try:
            if ser.in_waiting > 0:
                if ser.read(1) == bytes([HEADER]):
                    f_code = ser.read(1)
                    if f_code == bytes([FUNC_INCOMING_DATA]):
                        packet_data = ser.read(32)
                        if len(packet_data) == 32:
                            parsed_info = parse_stream_data(packet_data, logger=logger)
                            if parsed_info:
                                read_queue.put(parsed_info)
            try:
                packet_to_send = send_queue.get_nowait()
                ser.write(packet_to_send)
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
    last_print_time = time.time()
    while not shutdown_event.is_set():
        try:
            if socket.poll(100):
                topic, command_json = socket.recv_multipart()
                command = json.loads(command_json)
                latest_command.set_command(command)
                if time.time() - last_print_time > 0.25:
                    logger.debug(f"Received command: {command}")
                    last_print_time = time.time()
        except (zmq.ZMQError, json.JSONDecodeError) as e:
            logger.error(f"Error in command subscriber: {e}", exc_info=True)
            shutdown_event.set()
            break
    socket.close()
    logger.info("Command subscriber thread finished.")

def control_sender_thread(latest_command, send_queue, shutdown_event, logger):
    logger.info(f"Control sender started. Commands at {DOWNSAMPLE_RATE} Hz.")
    last_brake_force = 0
    while not shutdown_event.is_set():
        try:
            current_speed_rpm, current_steer_angle, time_stopped = latest_command.get_command()
            steer_packet = create_packet(FUNC_STEER, '<b', int(current_steer_angle))
            send_queue.put(steer_packet)
            brake_force_to_log = 0
            if current_speed_rpm != 0:
                if last_brake_force > 0:
                    send_queue.put(create_packet(FUNC_BRAKE, '<B', 0))
                    last_brake_force = 0
                if current_speed_rpm > 0:
                    speed_packet = create_packet(FUNC_DRIVE, '<h', int(current_speed_rpm * 10))
                else:
                    speed_packet = create_packet(FUNC_REVERSE, '<h', int(abs(current_speed_rpm) * 10))
                send_queue.put(speed_packet)
            else:
                if time_stopped is not None:
                    elapsed_time = time.time() - time_stopped
                    brake_force = min(MAX_BRAKE_FORCE, int(elapsed_time * BRAKE_RAMP_RATE))
                    send_queue.put(create_packet(FUNC_BRAKE, '<B', brake_force))
                    last_brake_force = brake_force
                    brake_force_to_log = brake_force
                else:
                    send_queue.put(create_packet(FUNC_BRAKE, '<B', 5))
                    last_brake_force = 5
                    brake_force_to_log = 5
            logger.debug(f"SENT > Speed: {current_speed_rpm:<5.1f} RPM, Steer: {current_steer_angle:<3.0f}, Brake: {brake_force_to_log}%")
            time.sleep(1.0 / DOWNSAMPLE_RATE)
        except Exception as e:
            logger.error(f"Error in control sender: {e}", exc_info=True)
            shutdown_event.set()
            break
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


# --- Main Managed Node Class ---
class LLCInterfaceNode(ManagedNode):
    """
    A managed node for the Low-Level Control (LLC) interface.
    Handles serial communication with the vehicle, publishes sensor data,
    and receives driving commands.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        self.ser = None
        self.read_queue = Queue()
        self.send_queue = Queue()
        self.latest_command = LatestCommand()
        self.threads = [] # Overwrites the one in parent to be specific to this class
        
        # We use the context from the parent class (ManagedNode)
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
        # Clear any stale commands in the queues
        while not self.send_queue.empty(): self.send_queue.get_nowait()
        
        # Start the data stream from the vehicle
        self.send_queue.put(create_packet(FUNC_STOP_STREAM))
        time.sleep(0.1)
        self.send_queue.put(create_packet(FUNC_START_STREAM))
        self.logger.info("Requested data stream start from vehicle.")

        # Create and start threads
        # The shutdown_event is inherited from the ManagedNode parent class
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

        # Signal all threads to stop
        # The parent's run loop will set this, but we can be explicit
        self.shutdown_event.set()

        # Stop the data stream and apply brake as a safety measure
        if self.ser and self.ser.is_open:
            try:
                self.logger.info("Stopping data stream and applying full brake.")
                self.ser.write(create_packet(FUNC_STOP_STREAM))
                time.sleep(0.1)
                self.ser.write(create_packet(FUNC_BRAKE, '<B', 100))
                time.sleep(0.1)
            except serial.SerialException as e:
                self.logger.error(f"Error sending deactivation commands: {e}")

        # Wait for threads to finish
        for t in self.threads:
            t.join(timeout=1.5)
        self.logger.info("All threads joined.")
        
        # Reset shutdown event in case we are reactivated later
        self.shutdown_event.clear()
        return True

    def on_shutdown(self) -> bool:
        self.logger.info("Shutting down LLC Interface Node...")
        # Deactivation already handles stopping threads.
        # Ensure final commands are sent and resources are closed.
        if self.ser and self.ser.is_open:
            try:
                self.logger.info("Applying full brake and centering steer on exit.")
                self.ser.write(create_packet(FUNC_BRAKE, '<B', 100))
                self.ser.write(create_packet(FUNC_STEER, '<b', 0))
                time.sleep(0.2)
            except serial.SerialException as e:
                self.logger.error(f"Error sending final commands: {e}")
            finally:
                self.ser.close()
                self.logger.info("Serial port closed.")
        return True

if __name__ == "__main__":
    # Setup logging
    log_filename = f"llc_managed_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s',
        filename=log_filename,
        filemode='w'
    )
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logging.getLogger().addHandler(console_handler)

    # Create and run the node
    llc_node = LLCInterfaceNode(node_name="llc_interface")
    llc_node.run()