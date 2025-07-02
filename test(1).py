import serial
import struct
import sys
import termios
import tty
import threading
import time
from typing import Dict, Any, Optional, List

class DataParser:
    """
    Parses incoming data packets from the serial stream based on their function code.
    """

    def parse_stream_data(self, data: bytes) -> Optional[Dict[str, Any]]:
        """
        Parses a stream data packet (function code 0x11).
        """
        if len(data) != 31:
            print(f"Warning: Expected 31 bytes for stream data payload, but got {len(data)}")
            return None

        format_string = '<bBhbbhhhhhhbbbBBff'
        if len(data) != struct.calcsize(format_string):
            print(f"Warning: Data length ({len(data)}) does not match format string length ({struct.calcsize(format_string)}).")
            return None

        try:
            parsed_data = struct.unpack(format_string, data)
            return {
                "battery_percentage": parsed_data[0],
                "charging_status": "Active" if parsed_data[1] == 0x01 else "Inactive",
                "rpm_roda": parsed_data[2] / 10.0,
                "brake_percentage": parsed_data[3],
                "steer_angle": parsed_data[4],
                "imu_x": parsed_data[5],
                "imu_y": parsed_data[6],
                "imu_z": parsed_data[7],
                "imu_yaw": parsed_data[8],
                "imu_pitch": parsed_data[9],
                "imu_roll": parsed_data[10],
                "radar_distance": parsed_data[11] / 100.0,
                "ultrasonic_front_distance": parsed_data[12] / 100.0,
                "ultrasonic_rear_distance": parsed_data[13] / 100.0,
                "bumper_switch_front": "Active" if parsed_data[14] == 1 else "Inactive",
                "bumper_switch_rear": "Active" if parsed_data[15] == 1 else "Inactive",
                "gps_latitude": parsed_data[16],
                "gps_longitude": parsed_data[17]
            }
        except struct.error as e:
            print(f"Error unpacking stream data: {e}")
            return None

    def parse_ack(self, data: bytes) -> Optional[Dict[str, Any]]:
        """
        Parses an acknowledgment packet.
        """
        # Assuming ACK packet is: [function_code_acked, status]
        if len(data) >= 2:
            return {
                "acknowledged_command": hex(data[0]),
                "status": "Success" if data[1] == 0x01 else "Failure"
            }
        return None

    def parse_data_request(self, data: bytes) -> Optional[Dict[str, Any]]:
        """
        Parses a response to a data request (function code 0x12).
        """
        # This is a placeholder. Implement according to your data request response format.
        print(f"Parsing data request response (0x12): {data.hex()}")
        return {"requested_data": data.hex()}

class SerialCommunicator:
    """
    Handles serial communication, including sending commands and
    running a background thread to parse incoming data.
    """

    def __init__(self, port: str, baudrate: int, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_port = None
        self.running = False
        self.parser = DataParser()
        self.latest_data = {}
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._read_from_port, daemon=True)

    def connect(self) -> bool:
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Successfully connected to {self.port}.")
            self.running = True
            self.thread.start()
            return True
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        if self.serial_port and self.serial_port.is_open:
            self.send_command([0x00]) # Stop streaming command
            self.serial_port.close()
            print("Serial port closed.")

    def _read_from_port(self):
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    start_byte = self.serial_port.read(1)
                    if start_byte == b'\xA5':
                        header_and_func_code = self.serial_port.read(2)
                        if len(header_and_func_code) == 2:
                            func_code = header_and_func_code[1]
                            
                            # Determine packet length based on function code
                            if func_code == 0x11:  # Stream data
                                packet_len = 33
                            elif func_code == 0x12: # Data request response
                                # Modify with actual length
                                packet_len = 10 
                            elif func_code in [0x10, 0x13]: # ACK
                                # Modify with actual length
                                packet_len = 4 
                            else:
                                continue

                            full_packet = self.serial_port.read(packet_len - 3) # Read rest of the packet
                            
                            if len(full_packet) == packet_len - 3:
                                received_checksum = self.serial_port.read(1)[0]
                                entire_packet = header_and_func_code + full_packet
                                calculated_checksum = sum(entire_packet) & 0xFF
                                
                                if calculated_checksum == received_checksum:
                                    payload = entire_packet[2:]
                                    self._dispatch_packet(func_code, payload)
                                else:
                                    print("Checksum mismatch!")
            except Exception as e:
                if self.running:
                    print(f"Error in reading thread: {e}")

    def _dispatch_packet(self, func_code: int, data: bytes):
        parsed_result = None
        if func_code == 0x11:
            parsed_result = self.parser.parse_stream_data(data)
            if parsed_result:
                with self.lock:
                    self.latest_data.update(parsed_result)
        elif func_code == 0x12:
            parsed_result = self.parser.parse_data_request(data)
        elif func_code in [0x10, 0x13]:
            parsed_result = self.parser.parse_ack(data)

        if parsed_result:
            print(f"Received (0x{func_code:02x}): {parsed_result}")


    def send_command(self, command_payload: List[int]):
        if not self.serial_port or not self.serial_port.is_open:
            print("Serial port not available.")
            return

        packet = [0xA5] + command_payload
        checksum = sum(packet) & 0xFF
        full_message = bytes(packet + [checksum])
        
        self.serial_port.write(full_message)
        print(f"Sent: {full_message.hex()}")

    def get_latest_data(self) -> Dict[str, Any]:
        with self.lock:
            return self.latest_data.copy()

class TeleopController:
    """
    Manages user input for teleoperation and sends commands.
    """

    def __init__(self, communicator: SerialCommunicator):
        self.communicator = communicator
        self.current_speed = 0
        self.current_steer = 0
        self.old_termios = None

    def _set_keypress_mode(self):
        self.old_termios = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

    def _restore_keypress_mode(self):
        if self.old_termios:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_termios)

    def _getch_non_blocking(self):
        import select
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def run(self):
        self._set_keypress_mode()
        print("Vehicle Teleoperation Started. Press 'q' to quit.")
        print("Controls: W: Forward, S: Reverse, A: Left, D: Right, X: Brake, Space: Stop")
        
        # Start streaming data
        self.communicator.send_command([0x01])

        try:
            while True:
                key = self._getch_non_blocking()
                if key:
                    if not self._handle_key_press(key):
                        break
                time.sleep(0.1)
        finally:
            self._restore_keypress_mode()
            print("Teleoperation stopped.")

    def _handle_key_press(self, key: str) -> bool:
        if key == 'q':
            return False
        elif key == 'w':
            self.current_speed = min(10000, self.current_speed + 100)
            self.communicator.send_command([0x03, 0x00, (self.current_speed >> 8) & 0xFF, self.current_speed & 0xFF])
        elif key == 's':
            self.current_speed = max(-10000, self.current_speed - 100)
            self.communicator.send_command([0x04, 0x00, (abs(self.current_speed) >> 8) & 0xFF, abs(self.current_speed) & 0xFF])
        elif key == 'a':
            self.current_steer = max(-30, self.current_steer - 1)
            steer_byte = int(128 - (self.current_steer / 30.0) * (128 - 90) if self.current_steer >= 0 else 128 - (self.current_steer / 30.0) * (166-128))
            self.communicator.send_command([0x06, max(90, min(166, steer_byte))])
        elif key == 'd':
            self.current_steer = min(30, self.current_steer + 1)
            steer_byte = int(128 - (self.current_steer / 30.0) * (128 - 90) if self.current_steer >= 0 else 128 - (self.current_steer / 30.0) * (166-128))
            self.communicator.send_command([0x06, max(90, min(166, steer_byte))])
        elif key == 'x':
            self.current_speed = 0
            self.communicator.send_command([0x05, 0x64])
        elif key == ' ':
            self.current_speed = 0
            self.current_steer = 0
            self.communicator.send_command([0x05, 0x00])
            self.communicator.send_command([0x03, 0x00, 0x00, 0x00])
            self.communicator.send_command([0x06, 0x80])
        return True

def main():
    SERIAL_PORT = '/dev/ttyACM0'
    BAUDRATE = 115200

    communicator = SerialCommunicator(port=SERIAL_PORT, baudrate=BAUDRATE)
    
    if not communicator.connect():
        sys.exit(1)

    controller = TeleopController(communicator)

    try:
        controller.run()
    except (KeyboardInterrupt, SystemExit):
        print("Shutting down...")
    finally:
        communicator.disconnect()

if __name__ == '__main__':
    main()