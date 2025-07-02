import serial
import sys
import struct
import tty
import termios
import time # Import time for small delays

# Your existing send_stream function
def send_stream(req: list, ser: serial.Serial):
    command_packet = req

    # --- Checksum Calculation ---
    # Sum all bytes in the command packet
    checksum = sum(command_packet) & 0xFF # Use & 0xFF to keep it an 8-bit value

    # --- Construct the Full Message ---
    # Append the checksum to the original packet
    full_message = command_packet + [checksum]

    # Convert the list of integers into a bytes object
    bytes_to_send = bytes(full_message)
    print(f"Sending: {bytes_to_send.hex()}") # Print what's being sent

    for byte in bytes_to_send:
        ser.write(byte.to_bytes(1, byteorder='big'))

# Function to set up non-blocking key input
def set_keypress_mode():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        return old_settings
    except termios.error as e:
        print(f"Error setting raw mode: {e}. Standard input might not work correctly.", file=sys.stderr)
        return old_settings

# Function to restore keypress mode
def restore_keypress_mode(old_settings):
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# Function to read a single character non-blocking
def getch_non_blocking():
    import select
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def main():
    old_termios_settings = set_keypress_mode()
    ser = None # Initialize ser to None

    print("Vehicle Teleoperation (Press 'q' to quit)")
    print("Controls: W: Forward, S: Reverse, A: Left, D: Right, X: Brake, Space: Stop")
    print("-" * 50)
    print("Waiting for vehicle data...")

    try:
        # Open serial port
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05) # Shorter timeout for faster response
        print(f"Listening on {ser.name}...")

        # Initial stream start
        send_stream([0xA5, 0x01], ser) # Start streaming
        print("Stream started.")

        current_speed = 0 # RPM * 10
        current_steer = 0 # degrees

        # Main teleoperation loop
        running = True
        while running:
            key = getch_non_blocking() # Read a character non-blocking

            # Process key input
            if key is not None: # A key was pressed
                if key == 'q':
                    running = False
                elif key == 'w': # Forward
                    current_speed = min(10000, current_speed + 100) # Max 1000 RPM (10000 * 0.1)
                    command_packet = [0xA5, 0x03, 0x00, (current_speed >> 8) & 0xFF, current_speed & 0xFF] # Drive command
                    send_stream(command_packet, ser)
                    print(f"Set speed: {current_speed / 10.0:.1f} RPM")
                elif key == 's': # Reverse
                    current_speed = max(-10000, current_speed - 100) # Min -1000 RPM
                    command_packet = [0xA5, 0x04, 0x00, (abs(current_speed) >> 8) & 0xFF, abs(current_speed) & 0xFF] # Reverse command
                    send_stream(command_packet, ser)
                    print(f"Set speed: {current_speed / 10.0:.1f} RPM")
                elif key == 'a': # Turn Left
                    current_steer = max(-30, current_steer - 1)
                    # Mapping -30 to 0xA6 (166), 30 to 0x5A (90), 0 to 0x80 (128)
                    steer_byte = int(128 - (current_steer / 30.0) * (128 - 90)) if current_steer >= 0 else int(128 - (current_steer / 30.0) * (166 - 128))
                    steer_byte = max(90, min(166, steer_byte)) # Clamp
                    command_packet = [0xA5, 0x06, steer_byte] # Set steer angle
                    send_stream(command_packet, ser)
                    print(f"Set steer angle: {current_steer} degrees")
                elif key == 'd': # Turn Right
                    current_steer = min(30, current_steer + 1)
                    steer_byte = int(128 - (current_steer / 30.0) * (128 - 90)) if current_steer >= 0 else int(128 - (current_steer / 30.0) * (166 - 128))
                    steer_byte = max(90, min(166, steer_byte)) # Clamp
                    command_packet = [0xA5, 0x06, steer_byte] # Set steer angle
                    send_stream(command_packet, ser)
                    print(f"Set steer angle: {current_steer} degrees")
                elif key == 'x': # Brake
                    current_speed = 0
                    command_packet = [0xA5, 0x05, 0x64] # 100% brake
                    send_stream(command_packet, ser)
                    print("Braking (100%)")
                elif key == ' ': # Stop (reset speed and steer)
                    current_speed = 0
                    current_steer = 0
                    send_stream([0xA5, 0x05, 0x00], ser) # Release brake
                    send_stream([0xA5, 0x03, 0x00, 0x00, 0x00], ser) # Stop drive
                    send_stream([0xA5, 0x06, 0x80], ser) # Center steer (assuming 0x80 is center)
                    print("Stopped and centered.")
                else:
                    print(f"Unrecognized key: {key}")

            # Read and parse serial data
            bytes_received = b''
            try:
                line = ser.read(1)
                if line == b'\xA5':  # Check if the first byte is 0xA5
                    bytes_received = ser.read(32)  # Read the next 32 bytes
                    if len(bytes_received) == 32:
                        # Checksum validation
                        checksum = sum(bytes_received[:-1]) & 0xFF
                        if checksum == bytes_received[-1]:
                            data = bytes_received[1:-1] # Remove command ID (0x11) and checksum
                            try:
                                # Adjusted struct.unpack format for 29 bytes of data
                                parsed_data = struct.unpack('<bBHbbhhhhhhbbbbbBBff', data)
                                # Print parsed data to console
                                print("\n--- Vehicle Status ---")
                                print(f"Battery: {parsed_data[0]}%")
                                print(f"Charging: {'Yes' if parsed_data[1] == 1 else 'No'}")
                                print(f"RPM: {parsed_data[2] / 10.0:.1f}")
                                print(f"Brake: {parsed_data[3]}%")
                                print(f"Steer Angle: {parsed_data[4]}")
                                print(f"IMU Yaw: {parsed_data[8] / 100.0:.2f}")
                                print(f"Radar Dist: {parsed_data[11] / 100.0:.2f} m")
                                # Add more as needed
                                print("----------------------")

                            except struct.error as e:
                                print(f"Struct unpack error: {e}. Check data format.", file=sys.stderr)
                        else:
                            print(f"Checksum invalid :{bytes_received[-1]} \r\n", file=sys.stderr)
                    else:
                        print(f"Incomplete data received: {len(bytes_received)} bytes.", file=sys.stderr)
                elif line: # If not 0xA5 but some other byte, print for debugging
                    # print(f"Received unexpected byte: {line.hex()}", file=sys.stderr)
                    pass # Suppress frequent unexpected byte prints

            except serial.SerialTimeoutException:
                pass # No data yet, continue loop
            except Exception as e:
                print(f"Error reading serial: {e}", file=sys.stderr)

            time.sleep(0.01) # Small delay to prevent busy-waiting

    except serial.SerialException as e:
        print(f"Error: Could not open serial port: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nProgram stopped by user.", file=sys.stderr)
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)
    finally:
        if ser is not None and ser.is_open:
            send_stream([0xA5, 0x00], ser) # Stop streaming on exit
            ser.close()
            print(f"Serial port {ser.name} closed.")
        print("Program stopped.")
        restore_keypress_mode(old_termios_settings)

if __name__ == '__main__':
    main()