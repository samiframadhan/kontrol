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
ZMQ_SUB_URL = "tcp://localhost:5555" # Receives commands from teleop
ZMQ_PUB_URL = "tcp://*:5556"         # Publishes sensor data

# --- Function Codes ---
FUNC_STOP_STREAM = 0x00
FUNC_START_STREAM = 0x01
FUNC_DRIVE = 0x03
FUNC_REVERSE = 0x04
FUNC_BRAKE = 0x05
FUNC_STEER = 0x06
FUNC_INCOMING_DATA = 0x11

# --- Packet Creation & Parsing ---
def create_packet(func_code, payload_format=None, value=None):
    """Constructs a complete binary packet."""
    if payload_format and value is not None:
        payload = struct.pack(payload_format, value)
    else:
        payload = b''
    packet_without_checksum = bytearray([HEADER, func_code]) + payload
    checksum = sum(packet_without_checksum) & 0xFF
    return packet_without_checksum + bytearray([checksum])

def parse_stream_data(bytes_received):
    """Parses the incoming 33-byte data stream from the device."""
    if len(bytes_received) != 32:
        logging.warning(f"Incorrect packet length received. Expected 32, got {len(bytes_received)}.")
        return None

    full_packet_for_checksum = bytearray([HEADER, FUNC_INCOMING_DATA]) + bytes_received[:-1]
    checksum_calculated = sum(full_packet_for_checksum) & 0xFF
    if checksum_calculated != bytes_received[-1]:
        logging.warning(f"Checksum mismatch! Calculated: {hex(checksum_calculated)}, Received: {hex(bytes_received[-1])}")
        return None

    data = bytes_received[:-1]
    format_string = '>BcHBbhhhhhhBBBccff'
    if len(data) != struct.calcsize(format_string):
        logging.error(f"Data length ({len(data)}) does not match format string size ({struct.calcsize(format_string)}).")
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
        logging.error(f"Failed to unpack data: {e}")
        return None

def update_speed_and_steer(command, current_speed_rpm, current_steer_angle):
    """Updates speed and steering angle based on the command."""
    speed_rpm = command.get("speed_rpm", current_speed_rpm)
    steer_angle = command.get("steer_angle", current_steer_angle)
    speed_rpm = max(-500.0, min(500.0, speed_rpm))
    steer_angle = max(-90.0, min(90.0, steer_angle))
    return speed_rpm, steer_angle

# --- Threads ---
def serial_io_thread(ser, read_queue, send_queue, shutdown_event):
    """
    Handles all serial communication.
    Reads from the serial port and puts parsed data onto the read_queue.
    Gets data from the send_queue and writes it to the serial port.
    """
    logging.info("Serial I/O thread started.")
    try:
        ser.write(create_packet(FUNC_STOP_STREAM))
        time.sleep(0.1)
        ser.write(create_packet(FUNC_START_STREAM))
        time.sleep(0.1)
    except serial.SerialException as e:
        logging.error(f"Failed to start data stream: {e}")
        shutdown_event.set()

    while not shutdown_event.is_set():
        try:
            # Read from serial and put on read_queue
            if ser.in_waiting > 0:
                if ser.read(1) == bytes([HEADER]):
                    f_code = ser.read(1)
                    if f_code == bytes([FUNC_INCOMING_DATA]):
                        packet_data = ser.read(32)
                        if len(packet_data) == 32:
                            parsed_info = parse_stream_data(packet_data)
                            if parsed_info:
                                read_queue.put(parsed_info)

            # Get from send_queue and write to serial
            try:
                packet_to_send = send_queue.get_nowait()
                ser.write(packet_to_send)
                send_queue.task_done()
            except Empty:
                pass # No outbound data to send

            time.sleep(0.005) # Prevent high CPU usage

        except (serial.SerialException, IOError) as e:
            logging.error(f"Serial error in I/O thread: {e}", exc_info=True)
            shutdown_event.set()
            break
    logging.info("Serial I/O thread finished.")


def command_subscriber(send_queue, context, shutdown_event):
    """Subscribes to ZMQ commands and places them on the send_queue."""
    socket = context.socket(zmq.SUB)
    socket.connect(ZMQ_SUB_URL)
    socket.setsockopt_string(zmq.SUBSCRIBE, "teleop_cmd")
    logging.info(f"Listening for commands on {ZMQ_SUB_URL}")
    current_speed_rpm = 0.0
    current_steer_angle = 0.0

    while not shutdown_event.is_set():
        try:
            if socket.poll(1000):
                topic, command_json = socket.recv_multipart()
                command = json.loads(command_json)

                current_speed_rpm, current_steer_angle = update_speed_and_steer(
                    command, current_speed_rpm, current_steer_angle)

                if current_speed_rpm > 0:
                    packet = create_packet(FUNC_DRIVE, '<h', int(current_speed_rpm * 10))
                elif current_speed_rpm < 0:
                    packet = create_packet(FUNC_REVERSE, '<h', int(abs(current_speed_rpm) * 10))
                else:
                    packet = create_packet(FUNC_BRAKE, '<B', 0)
                send_queue.put(packet)

                steer_packet = create_packet(FUNC_STEER, '<b', int(current_steer_angle))
                send_queue.put(steer_packet)

                print(f"\rSENT > Speed: {current_speed_rpm:<5.1f}, Steer: {current_steer_angle:<3.0f}", end="", flush=True)

        except (zmq.ZMQError, json.JSONDecodeError) as e:
            logging.error(f"Error in command subscriber: {e}", exc_info=True)
            shutdown_event.set()
            break
    logging.info("Command subscriber thread finished.")


def data_publisher(read_queue, context, shutdown_event):
    """Gets data from the read_queue and publishes it via ZMQ."""
    socket = context.socket(zmq.PUB)
    socket.bind(ZMQ_PUB_URL)
    logging.info(f"Publishing sensor data on {ZMQ_PUB_URL}")

    while not shutdown_event.is_set():
        try:
            # Get data from the queue with a timeout to prevent blocking
            parsed_info = read_queue.get(timeout=1)
            socket.send_string("sensor_data", flags=zmq.SNDMORE)
            socket.send_json(parsed_info)
            read_queue.task_done()
        except Empty:
            # This is expected if no data comes from serial for 1s
            continue
        except Exception as e:
            logging.error(f"An unexpected error occurred in data publisher: {e}", exc_info=True)
            shutdown_event.set()
            break
    logging.info("Data publisher thread finished.")


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
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logging.getLogger().addHandler(console_handler)

    context = zmq.Context()
    # Create shared queues
    read_queue = Queue()
    send_queue = Queue()
    shutdown_event = threading.Event()

    ser = None
    threads = []

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        logging.info(f"Successfully opened serial port {SERIAL_PORT}")

        # --- Create and start threads ---
        io_thread = threading.Thread(target=serial_io_thread, args=(ser, read_queue, send_queue, shutdown_event), name="SerialIOThread")
        sub_thread = threading.Thread(target=command_subscriber, args=(send_queue, context, shutdown_event), name="CommandSubThread")
        pub_thread = threading.Thread(target=data_publisher, args=(read_queue, context, shutdown_event), name="DataPubThread")
        
        threads.extend([io_thread, sub_thread, pub_thread])

        for t in threads:
            t.daemon = True
            t.start()
        
        shutdown_event.wait()

    except serial.SerialException as e:
        logging.error(f"Could not open serial port '{SERIAL_PORT}'. {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        logging.info("Caught KeyboardInterrupt, shutting down...")
    finally:
        logging.info("Signaling threads to terminate and closing resources.")
        shutdown_event.set()
        
        for t in threads:
            t.join(timeout=1.5)

        if ser and ser.is_open:
            try:
                logging.info("Applying full brake.")
                ser.write(create_packet(FUNC_BRAKE, '<B', 100))
                ser.write(create_packet(FUNC_STEER, '<b', 0))
                time.sleep(0.2)
            except serial.SerialException as e:
                logging.error(f"Error sending final commands: {e}")
            finally:
                ser.close()
                logging.info("Serial port closed.")
        
        context.term()
        logging.info("ZMQ context terminated.")

if __name__ == "__main__":
    main()