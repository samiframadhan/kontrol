import serial
import sys
import struct

def parse_stream_data(bytes_received):
    """
    Parses a byte string containing stream data based on the provided image specification.
    """
    if len(bytes_received) != 33:
        print(f"Warning: Expected 32 bytes for stream data, but got {len(bytes_received)}")
        return None
    
    # Check if the first byte is the expected function code (0x11)
    if bytes_received[0] != 0x11:
        print(f"Warning: Expected function code 0x11, but got {bytes_received[0]:#04x}")
        print(f"Data received: {bytes_received.hex()}\n\r")
        return None

    # Checksum validation (including the initial 0xA5 header and 0x11 function code)
    # Assume the header byte (0xA5) is available as a parameter or global variable.
    # If not, you may need to pass it in or define it here.
    HEADER_BYTE = 0xA5
    checksum_calculated = (HEADER_BYTE + sum(bytes_received[:-1])) & 0xFF
    checksum_received = bytes_received[-1]

    if checksum_calculated != checksum_received:
        print(f"Checksum mismatch! Calculated: {checksum_calculated:#04x}, Received: {checksum_received:#04x}")
        print(f"Data received: {bytes_received.hex()}\n\r")
        return None

    print("Checksum is valid.")

    # Extract the actual data (excluding function code and checksum)
    data = bytes_received[1:-1] # From index 1 (after 0x11) to the second to last byte (before checksum)

    # Define the unpack format string based on the image:
    # No. | Register Name                       | Byte No | Type    | Bytes | Description
    # ----|--------------------------           |---------|---------|-------|----------------------------------------------------
    # 2   | Battery percentage                  | 0       | int8    | 1     |
    # 3   | Charging status                     | 1       | char    | 1     | 0x00 inactive, 0x01 active
    # 4   | RPM roda                            | 2       | int16   | 2     | 0x00 hingga 0x2710 atau 0-10000, dibagi 10 (0.0-1000.0)
    # 5   | Brake percentage                    | 4       | int8    | 1     | 0x00 hingga 0x64 atau 0-100
    # 6   | Steer angle                         | 5       | int8    | 1     | 0xA6 hingga 0x5A atau -90 - +90
    # 7   | IMU x                               | 6       | int16   | 2     |
    # 8   | IMU y                               | 8       | int16   | 2     |
    # 9   | IMU z                               | 10      | int16   | 2     |
    # 10  | IMU yaw                             | 12      | int16   | 2     |
    # 11  | IMU pitch                           | 14      | int16   | 2     |
    # 12  | IMU roll                            | 16      | int16   | 2     |
    # 13  | Radar objek terdekat                | 18      | int8    | 1     | 0x00 hingga 0xFF atau 0 hingga 255, dibagi 100 (0-2.55 m)
    # 14  | Ultrasonic depan objek terdekat     | 19      | int8    | 1     | 0x00 hingga 0xFF atau 0 hingga 255, dibagi 100 (0-2.55 m)
    # 15  | Ultrasonic belakang objek terdekat  | 20      | int8    | 1     | 0x00 hingga 0xFF atau 0 hingga 255, dibagi 100 (0-2.55 m)
    # 16  | Bumper switch depan                 | 21      | char    | 1     | 0 --> tidak aktif, 1 --> aktif
    # 17  | Bumper switch belakang              | 22      | char    | 1     | 0 --> tidak aktif, 1 --> aktif
    # 18  | GPS latitude                        | 23      | float   | 4     |
    # 19  | GPS longitude                       | 27      | float   | 4     |

    format_string = '<bBhbbhhhhhhbbbBBff' # 1 + 1 + 2 + 1 + 1 + 6*2 + 3*1 + 2*1 + 2*4 = 1+1+2+1+1+12+3+2+8 = 31 bytes

    try:
        parsed_data = struct.unpack(format_string, data)

        # Assign names for clarity
        parsed_info = {
            "battery_percentage": parsed_data[0],
            "charging_status": parsed_data[1],
            "rpm_roda": parsed_data[2] / 10.0, # Divide by 10 as per description
            "brake_percentage": parsed_data[3],
            "steer_angle": parsed_data[4], # Need to handle 0xA6 to 0x5A mapping
            "imu_x": parsed_data[5],
            "imu_y": parsed_data[6],
            "imu_z": parsed_data[7],
            "imu_yaw": parsed_data[8],
            "imu_pitch": parsed_data[9],
            "imu_roll": parsed_data[10],
            "radar_distance": parsed_data[11] / 100.0, # Divide by 100 as per description
            "ultrasonic_front_distance": parsed_data[12] / 100.0, # Divide by 100
            "ultrasonic_rear_distance": parsed_data[13] / 100.0, # Divide by 100
            "bumper_switch_front": parsed_data[14], # 0: inactive, 1: active
            "bumper_switch_rear": parsed_data[15],   # 0: inactive, 1: active
            "gps_latitude": parsed_data[16],
            "gps_longitude": parsed_data[17]
        }

        # Special handling for steer angle: 0xA6 to 0x5A maps to -90 to +90
        # If the steer angle is received as a raw byte, it might be an unsigned value.
        # Python's `struct.unpack` with 'b' (signed char) will handle the negative values if it's two's complement.
        # The range 0xA6 (-90) to 0x5A (90) implies signed byte interpretation.
        # So parsed_info["steer_angle"] should already be correct.

        # Special handling for GPS data if it needs further conversion (from hex to decimal, then divided)
        # The description says "Dari Hexadesimal 650CA000 hingga 460CA000 atau -18000.000 hingga +18000.000 dalam desimal.
        # Tiga digit pertama adalah derajat (0-180), digit selanjutnya adalah menit (00.000 hingga 59.999)"
        # This implies the float value itself represents the total degrees.
        # The Python `float` will directly interpret the 4 bytes. The description might be for a different level of parsing.

        return parsed_info

    except struct.error as e:
        print(f"Struct unpacking error: {e}")
        print(f"Data length: {len(data)}, Expected format string length: {struct.calcsize(format_string)}")
        return None

SERIAL_PORT = '/dev/ttyACM0'  # Change as needed
BAUDRATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

try:
    payload = [0xA5, 0x01]  # Start streaming command
    checksum = sum(payload) & 0xFF  # Calculate checksum
    payload.append(checksum)  # Append checksum to the payload
    ser.write(bytearray(payload))
    while True:
        # Read 32 bytes (one packet)
        packet = ser.read(1)
        if packet != b'\xA5':  # Check for start byte
            continue
        packet = ser.read(33)
        if len(packet) == 33:
            result = parse_stream_data(packet)
            if result:
                print(result)
        else:
            print("Incomplete packet received.")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
