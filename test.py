import serial
import time
import struct
import threading
import sys

# Platform-specific imports for reading single characters from the keyboard
try:
    import tty
    import termios
    import select
    UNIX = True
except ImportError:
    import msvcrt
    UNIX = False

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
HEADER = 0xA5
SEND_FREQUENCY_HZ = 5.0  # Send commands at 5 Hz

# --- Control Settings ---
MAX_SPEED_RPM = 500.0   # The maximum speed in RPM
MAX_STEER_ANGLE = 90.0  # The maximum steering angle in degrees
SPEED_STEP = 25.0       # How much to increase/decrease speed per key press
STEER_STEP = 5.0        # How much to adjust steering per key press

# --- Function Codes ---
FUNC_START_STREAM = 0x01
FUNC_DRIVE = 0x03
FUNC_REVERSE = 0x04
FUNC_BRAKE = 0x05
FUNC_STEER = 0x06
FUNC_INCOMING_DATA = 0x11

# --- Shared state for commands ---
latest_commands = {
    'motion': None,
    'steer': None,
}
state_lock = threading.Lock()

# --- Serial Port Initialization ---
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print(f"Successfully opened serial port {SERIAL_PORT} at {BAUDRATE} baud.")
except serial.SerialException as e:
    print(f"Error: Could not open serial port '{SERIAL_PORT}'. {e}")
    print("Please check the port name and ensure the device is connected.")
    sys.exit(1)

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
    # print(f"{hex(bytes_received)}")
    if len(bytes_received) != 32:
        return None
    
    full_packet_for_checksum = bytearray([HEADER]) + bytearray([0x11]) + bytes_received[:-1]
    checksum_calculated = sum(full_packet_for_checksum) & 0xFF
    checksum_received = bytes_received[-1]
    if checksum_calculated != checksum_received:
        print(f"\n[Parser Error] Checksum mismatch! Calculated: {checksum_calculated:#04x}, Received: {checksum_received:#04x}")
        return None
    else:
        print("\n\rchecksum match")

    data = bytes_received[:-1]
    format_string = '<bBhbbhhhhhhbbbBBff'
    if len(data) != struct.calcsize(format_string):
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
    except struct.error:
        print("failed to parse")
        return None

def command_sender():
    """A dedicated thread that sends the latest commands at a fixed frequency."""
    send_interval = 1.0 / SEND_FREQUENCY_HZ
    while True:
        with state_lock:
            motion_cmd = latest_commands['motion']
            steer_cmd = latest_commands['steer']
        if motion_cmd:
            ser.write(motion_cmd)
        if steer_cmd:
            ser.write(steer_cmd)
        time.sleep(send_interval)

def stream_data_reader():
    """A dedicated thread that continuously reads and parses data from the serial port."""
    global current_speed_rpm, current_steer_angle
    while True:
        try:
            if ser.read(1) == bytes([HEADER]):
                f_code = ser.read(1)
                if f_code == bytes([0x11]):
                    packet_data = ser.read(32)
                    if len(packet_data) == 32:
                        parsed_info = parse_stream_data(packet_data)
                        # Update the display line with both commanded and received data
                        cmd_status = f"CMD > Speed: {current_speed_rpm:<5.1f} RPM, Steer: {current_steer_angle:<3.0f} deg"
                        if parsed_info:
                            data_status = f"RECV > RPM: {parsed_info['rpm']:<5.1f} | Bat: {parsed_info['battery_%']}% | Steer: {parsed_info['steer_angle']:<3} | Brake {parsed_info['brake_%']}"
                            print(f"\r\n{cmd_status} | {data_status}", end="")
                        else:
                            print
                            print(f"\r\n{cmd_status}", end="")
                elif f_code == [0x12]:
                    print(f"Is it ack?{f_code}")
            else:
                continue

        except (serial.SerialException, Exception):
            break

def getKey(settings, timeout=0.1):
    """Gets a single character from standard input."""
    if UNIX:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    else:
        if msvcrt.kbhit():
            key = msvcrt.getch().decode('utf-8')
        else:
            key = ''
    return key

def save_terminal_settings():
    """Saves the current terminal settings."""
    if UNIX:
        return termios.tcgetattr(sys.stdin)
    return None

def restore_terminal_settings(old_settings):
    """Restores terminal settings to their original state."""
    if UNIX:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# Global variables for current commanded state
current_speed_rpm = 0.0
current_steer_angle = 0.0

def main():
    """The main function to run the teleoperation interface."""
    global current_speed_rpm, current_steer_angle
    settings = save_terminal_settings()

    sender_thread = threading.Thread(target=command_sender, daemon=True)
    sender_thread.start()
    receiver_thread = threading.Thread(target=stream_data_reader, daemon=True)
    receiver_thread.start()

    print("Sending command to start data stream...")
    ser.write(create_packet(FUNC_START_STREAM))
    time.sleep(0.1)

    print("\n--- Keyboard Teleoperation Control (Incremental) ---")
    print("       w: Increase Speed")
    print("a: Steer Left     s: Decrease Speed     d: Steer Right")
    print("       space: Brake (Set Speed to 0)")
    print("       x: Full Stop (Zero Speed & Steering)")
    print("-------------------------------------------------")
    print("Press 'q' to quit.\n")

    try:
        while True:
            key = getKey(settings)

            if key == 'w':
                current_speed_rpm = min(current_speed_rpm + SPEED_STEP, MAX_SPEED_RPM)
            elif key == 's':
                current_speed_rpm = max(current_speed_rpm - SPEED_STEP, -MAX_SPEED_RPM)
            elif key == 'a':
                current_steer_angle = max(current_steer_angle - STEER_STEP, -MAX_STEER_ANGLE)
            elif key == 'd':
                current_steer_angle = min(current_steer_angle + STEER_STEP, MAX_STEER_ANGLE)
            elif key == ' ':
                current_speed_rpm = 0.0
            elif key == 'x':
                current_speed_rpm = 0.0
                current_steer_angle = 0.0
            elif key == 'q':
                print("\nExiting program...")
                break

            # --- Create packets based on the current state ---
            new_motion_packet = None
            if current_speed_rpm > 0:
                # Positive speed -> Drive command
                rpm_val = int(current_speed_rpm * 10)
                new_motion_packet = create_packet(FUNC_DRIVE, '<h', rpm_val)
            elif current_speed_rpm < 0:
                # Negative speed -> Reverse command with absolute value
                rpm_val = int(abs(current_speed_rpm) * 10)
                new_motion_packet = create_packet(FUNC_REVERSE, '<h', rpm_val)
            else:
                # Zero speed -> Brake command
                new_motion_packet = create_packet(FUNC_BRAKE, '<B', 20)
            
            new_steer_packet = create_packet(FUNC_STEER, '<b', int(current_steer_angle))

            # Update the shared state for the sender thread
            with state_lock:
                latest_commands['motion'] = new_motion_packet
                latest_commands['steer'] = new_steer_packet
            
            # A small sleep to prevent this loop from running too fast
            time.sleep(0.05)

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        print("\nApplying full brake and closing.")
        ser.write(create_packet(FUNC_BRAKE, '<B', 100))
        ser.write(create_packet(FUNC_STEER, '<b', 0))
        time.sleep(0.2)
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")
        restore_terminal_settings(settings)

if __name__ == "__main__":
    main()
