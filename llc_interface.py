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
FUNC_DRIVE = 0x03
FUNC_REVERSE = 0x04
FUNC_BRAKE = 0x05
FUNC_STEER = 0x06
FUNC_INCOMING_DATA = 0x11
DOWNSAMPLE_RATE = 10  # Hz, the rate commands will be sent to the vehicle

# --- NEW: Braking Configuration ---
BRAKE_RAMP_RATE = 25  # Brake force percentage increase per second (e.g., 25 means 0-100% in 4 seconds)
MAX_BRAKE_FORCE = 100 # Maximum brake force percentage

# --- REVISED: Thread-safe object to store the latest command and stop time ---
class LatestCommand:
    """
    A thread-safe class to store the latest command, including the time
    the vehicle was commanded to stop.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.speed_rpm = 0.0
        self.steer_angle = 0.0
        self.time_stopped = None

    def set_command(self, command):
        """Safely update the command values and track stop time."""
        with self.lock:
            new_speed_rpm = command.get("speed_rpm", self.speed_rpm)

            # Check if the vehicle is being commanded to stop
            if new_speed_rpm == 0.0 and self.speed_rpm != 0.0:
                # If it wasn't already stopped, record the time
                if self.time_stopped is None:
                    self.time_stopped = time.time()
            # Check if the vehicle is being commanded to move
            elif new_speed_rpm != 0.0:
                # If it was stopped, reset the timer
                self.time_stopped = None

            new_steer_angle = command.get("steer_angle", self.steer_angle)
            if new_steer_angle == 0.0:
                new_steer_angle = self.steer_angle  # Maintain current steer angle if not specified
            
            self.steer_angle = new_steer_angle
            
            self.speed_rpm = new_speed_rpm

            # Clamp values to safe limits
            self.speed_rpm = max(-100.0, min(500.0, self.speed_rpm))
            self.steer_angle = max(-120.0, min(90.0, self.steer_angle))

    def get_command(self):
        """Safely retrieve the latest command values and stop time."""
        with self.lock:
            return self.speed_rpm, self.steer_angle, self.time_stopped

# --- Packet Creation & Parsing (Unchanged) ---
def create_packet(func_code, payload_format=None, value=None):
    """Constructs a complete binary packet."""
    if payload_format and value is not None:
        payload = struct.pack(payload_format, value)
    else:
        payload = b''
    packet_without_checksum = bytearray([HEADER, func_code]) + payload
    checksum = sum(packet_without_checksum) & 0xFF
    return packet_without_checksum + bytearray([checksum])

def parse_stream_data(bytes_received, logger):
    """Parses the incoming 33-byte data stream from the device."""
    if len(bytes_received) != 32:
        logger.warning(f"Incorrect packet length received. Expected 32, got {len(bytes_received)}.")
        return None

    full_packet_for_checksum = bytearray([HEADER, FUNC_INCOMING_DATA]) + bytes_received[:-1]
    checksum_calculated = sum(full_packet_for_checksum) & 0xFF
    if checksum_calculated != bytes_received[-1]:
        logger.warning(f"Checksum mismatch! Calculated: {hex(checksum_calculated)}, Received: {hex(bytes_received[-1])}")
        return None

    data = bytes_received[:-1]
    format_string = '>BcHBbhhhhhhBBBccff'
    if len(data) != struct.calcsize(format_string):
        logger.error(f"Data length ({len(data)}) does not match format string size ({struct.calcsize(format_string)}).")
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

# --- Threads ---
def serial_io_thread(ser, read_queue, send_queue, shutdown_event, logger):
    """
    Handles all serial communication. (Largely Unchanged)
    Reads from the serial port and puts parsed data onto the read_queue.
    Gets data from the send_queue and writes it to the serial port.
    """
    logger.info("Serial I/O thread started.")
    try:
        send_queue.put(create_packet(FUNC_STOP_STREAM))
        time.sleep(0.1)
        # send_queue.put(create_packet(FUNC_START_STREAM))
        # time.sleep(0.1)
    except serial.SerialException as e:
        logger.error(f"Failed to start data stream: {e}")
        shutdown_event.set()

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
    """Subscribes to ZMQ commands and updates the latest_command object."""
    socket = context.socket(zmq.SUB)
    socket.connect(ZMQ_SUB_URL)
    socket.setsockopt_string(zmq.SUBSCRIBE, "teleop_cmd")
    logger.info(f"Listening for commands on {ZMQ_SUB_URL}")
    
    last_print_time = time.time()

    while not shutdown_event.is_set():
        try:
            if socket.poll(100): # Poll with a timeout
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
    logger.info("Command subscriber thread finished.")

# --- REVISED: This thread sends commands, including gradual braking, to the vehicle.
def control_sender_thread(latest_command, send_queue, shutdown_event, logger):
    """Periodically sends the latest command to the serial queue with gradual braking logic."""
    logger.info(f"Control sender started. Commands at {DOWNSAMPLE_RATE} Hz. Brake ramp rate: {BRAKE_RAMP_RATE}%/s.")
    
    last_brake_force = 0

    while not shutdown_event.is_set():
        try:
            current_speed_rpm, current_steer_angle, time_stopped = latest_command.get_command()

            # Create and queue steering packet (independent of speed)
            steer_packet = create_packet(FUNC_STEER, '<b', int(current_steer_angle))
            send_queue.put(steer_packet)

            brake_force_to_log = 0

            # --- REVISED Braking and Speed Logic ---
            if current_speed_rpm != 0:
                # --- Moving: Release brake and send drive/reverse command ---
                
                # Explicitly release the brake if it was applied
                if last_brake_force > 0:
                    brake_packet = create_packet(FUNC_BRAKE, '<B', 0)
                    send_queue.put(brake_packet)
                    last_brake_force = 0
                
                # Send drive or reverse command
                if current_speed_rpm > 0:
                    speed_packet = create_packet(FUNC_DRIVE, '<h', int(current_speed_rpm * 10))
                else: # current_speed_rpm < 0
                    speed_packet = create_packet(FUNC_REVERSE, '<h', int(abs(current_speed_rpm) * 10))
                send_queue.put(speed_packet)

            else:
                # --- Stopped: Apply brake gradually ---
                if time_stopped is not None:
                    # Calculate how long we've been commanded to be stopped
                    elapsed_time = time.time() - time_stopped
                    
                    # Calculate brake force, ramping up over time
                    brake_force = min(MAX_BRAKE_FORCE, int(elapsed_time * BRAKE_RAMP_RATE))
                    
                    # Create and queue the brake packet
                    brake_packet = create_packet(FUNC_BRAKE, '<B', brake_force)
                    send_queue.put(brake_packet)
                    
                    last_brake_force = brake_force
                    brake_force_to_log = brake_force
                else:
                    # Fallback: If time_stopped isn't set yet, apply a minimum brake force
                    # to prevent rolling on initial stop command.
                    brake_packet = create_packet(FUNC_BRAKE, '<B', 5) 
                    send_queue.put(brake_packet)
                    last_brake_force = 5
                    brake_force_to_log = 5
            
            logger.debug(f"SENT > Speed: {current_speed_rpm:<5.1f} RPM, Steer: {current_steer_angle:<3.0f}, Brake: {brake_force_to_log}%")
            
            # Wait to maintain the desired sending rate
            time.sleep(1.0 / DOWNSAMPLE_RATE)

        except Exception as e:
            logger.error(f"An unexpected error occurred in control sender: {e}", exc_info=True)
            shutdown_event.set()
            break
    logger.info("Control sender thread finished.")


def data_publisher(read_queue, context, shutdown_event, logger):
    """Gets data from the read_queue and publishes it via ZMQ. (Unchanged)"""
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
            logger.error(f"An unexpected error occurred in data publisher: {e}", exc_info=True)
            shutdown_event.set()
            break
    logger.info("Data publisher thread finished.")


def main():
    """Main function to run the serial bridge."""
    log_filename = f"debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s',
        filename=log_filename,
        filemode='w'
    )
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO) # Set to INFO for cleaner console output
    console_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logger = logging.getLogger()
    logger.addHandler(console_handler)

    context = zmq.Context()
    read_queue = Queue()
    send_queue = Queue()
    shutdown_event = threading.Event()
    
    # --- Create the shared command object ---
    latest_command = LatestCommand()

    ser = None
    threads = []

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        logger.info(f"Successfully opened serial port {SERIAL_PORT}")

        # --- Create and start threads ---
        io_thread = threading.Thread(target=serial_io_thread, args=(ser, read_queue, send_queue, shutdown_event, logger), name="SerialIOThread")
        sub_thread = threading.Thread(target=command_subscriber, args=(latest_command, context, shutdown_event, logger), name="CommandSubThread")
        pub_thread = threading.Thread(target=data_publisher, args=(read_queue, context, shutdown_event, logger), name="DataPubThread")
        sender_thread = threading.Thread(target=control_sender_thread, args=(latest_command, send_queue, shutdown_event, logger), name="ControlSenderThread")
        
        threads.extend([io_thread, sub_thread, pub_thread, sender_thread])

        for t in threads:
            t.daemon = True
            t.start()
        
        shutdown_event.wait()

    except serial.SerialException as e:
        logger.error(f"Could not open serial port '{SERIAL_PORT}'. {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        logger.info("Caught KeyboardInterrupt, shutting down...")
    finally:
        logger.info("Signaling threads to terminate and closing resources.")
        shutdown_event.set()
        
        for t in threads:
            t.join(timeout=1.5)

        if ser and ser.is_open:
            try:
                logger.info("Applying full brake and centering steer on exit.")
                ser.write(create_packet(FUNC_BRAKE, '<B', 100))
                ser.write(create_packet(FUNC_STEER, '<b', 0))
                time.sleep(0.2)
            except serial.SerialException as e:
                logger.error(f"Error sending final commands: {e}")
            finally:
                ser.close()
                logger.info("Serial port closed.")
        
        context.term()
        logger.info("ZMQ context terminated.")

if __name__ == "__main__":
    main()