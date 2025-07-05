# serial_bridge.py
import serial
import struct
import threading
import time
import sys
import zmq
import json
import logging
from datetime import datetime

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'  # Change this to your port
BAUDRATE = 115200
HEADER = 0xA5

# --- ZMQ Configuration ---
ZMQ_SUB_URL = "tcp://localhost:5555" # Receives commands from teleop
ZMQ_PUB_URL = "tcp://*:5556"         # Publishes sensor data

# --- Function Codes (copied from original) ---
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
    # The actual data payload is 32 bytes, as the header and func code are handled separately.
    if len(bytes_received) != 32:
        logging.warning(f"Incorrect packet length received. Expected 32, got {len(bytes_received)}.")
        return None

    full_packet_for_checksum = bytearray([HEADER, FUNC_INCOMING_DATA]) + bytes_received[:-1]
    checksum_calculated = sum(full_packet_for_checksum) & 0xFF
    if checksum_calculated != bytes_received[-1]:
        logging.warning(f"Checksum mismatch! Calculated: {hex(checksum_calculated)}, Received: {hex(bytes_received[-1])}")
        return None

    data = bytes_received[:-1] # Exclude the checksum byte
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

# --- Threads ---
def command_subscriber(ser, context):
    """Subscribes to ZMQ commands and sends them to the serial port."""
    socket = context.socket(zmq.SUB)
    socket.connect(ZMQ_SUB_URL)
    socket.setsockopt_string(zmq.SUBSCRIBE, "teleop_cmd")
    logging.info(f"Listening for commands on {ZMQ_SUB_URL}")

    while True:
        try:
            # Read the topic and the JSON payload
            topic, command_json = socket.recv_multipart()
            command = json.loads(command_json)

            speed_rpm = command.get("speed_rpm", 0.0)
            steer_angle = command.get("steer_angle", 0.0)

            # --- Create and send packets ---
            if speed_rpm > 0:
                packet = create_packet(FUNC_DRIVE, '<h', int(speed_rpm * 10))
            elif speed_rpm < 0:
                packet = create_packet(FUNC_REVERSE, '<h', int(abs(speed_rpm) * 10))
            else:
                packet = create_packet(FUNC_BRAKE, '<B', 20) # Default brake
            ser.write(packet)
            time.sleep(0.005) # Small delay between packets

            steer_packet = create_packet(FUNC_STEER, '<b', int(steer_angle))
            ser.write(steer_packet)

            print(f"\rSENT > Speed: {speed_rpm:<5.1f}, Steer: {steer_angle:<3.0f}", end="", flush=True)

        except (zmq.ZMQError, json.JSONDecodeError, serial.SerialException) as e:
            logging.error(f"Error in command subscriber: {e}", exc_info=True)
            break

def data_publisher(ser, context):
    """Reads from serial and publishes data via ZMQ."""
    socket = context.socket(zmq.PUB)
    socket.bind(ZMQ_PUB_URL)
    logging.info(f"Publishing sensor data on {ZMQ_PUB_URL}")

    # Start the data stream from the device
    ser.write(create_packet(FUNC_STOP_STREAM))
    time.sleep(0.1)  # Give some time for the device to stop
    ser.write(create_packet(FUNC_START_STREAM))
    time.sleep(0.1)

    while True:
        try:
            if ser.in_waiting > 0 and ser.read(1) == bytes([HEADER]):
                f_code = ser.read(1)
                if f_code == bytes([FUNC_INCOMING_DATA]):
                    # Read the 32 bytes of the data payload + checksum
                    packet_data = ser.read(32)
                    if len(packet_data) == 32:
                        logging.debug(f"Received HEX: {packet_data.hex()}")
                        parsed_info = parse_stream_data(packet_data)
                        if parsed_info:
                            logging.debug(f"Parsed Value: {json.dumps(parsed_info)}")
                            # Publish data on the "sensor_data" topic
                            socket.send_string("sensor_data", flags=zmq.SNDMORE)
                            socket.send_json(parsed_info)
        except serial.SerialException as e:
            logging.error(f"Serial error in data publisher: {e}", exc_info=True)
            break
        except Exception as e:
            logging.error(f"An unexpected error occurred in data publisher: {e}", exc_info=True)
            time.sleep(1) # Avoid spamming errors


def main():
    """Main function to run the serial bridge."""
    # --- Setup Logging ---
    log_filename = f"debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s',
        filename=log_filename,
        filemode='w'
    )
    # Add a console handler to show INFO messages on the terminal
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.INFO)
    console_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_formatter)
    logging.getLogger().addHandler(console_handler)

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        logging.info(f"Successfully opened serial port {SERIAL_PORT}")
    except serial.SerialException as e:
        logging.error(f"Could not open serial port '{SERIAL_PORT}'. {e}")
        sys.exit(1)

    context = zmq.Context()

    sub_thread = threading.Thread(target=command_subscriber, args=(ser, context), name="CommandSubThread", daemon=True)
    pub_thread = threading.Thread(target=data_publisher, args=(ser, context), name="DataPubThread", daemon=True)

    sub_thread.start()
    pub_thread.start()

    try:
        # Keep the main thread alive to handle termination
        while sub_thread.is_alive() and pub_thread.is_alive():
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Caught KeyboardInterrupt, shutting down...")
    finally:
        logging.info("Applying full brake and closing resources.")
        try:
            ser.write(create_packet(FUNC_BRAKE, '<B', 100))
            ser.write(create_packet(FUNC_STEER, '<b', 0))
            time.sleep(0.2)
        except serial.SerialException as e:
            logging.error(f"Error sending final commands: {e}")
        finally:
            ser.close()
            context.term()
            logging.info("Serial port closed and ZMQ context terminated.")


if __name__ == "__main__":
    main()