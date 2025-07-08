import struct

#000001fe000400000000000000000000000000000000000000000000000000b9
# Example incoming data
# 32 bytes: 00 00 01 fe 00 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 b9
incoming_data = bytes.fromhex('a511000001fe000400000000000000000000000000000000000000000000000000b9')
# Byte sequence: # 0xA5 (header) + 0x11 (function code) + 32 bytes of data + 0xB9 (checksum)
# Data sequence: # 0x00 (battery %) + 0x01 (charging) + 0xFE (rpm) + 0x00 (brake %) + 0x04 (steer angle) + 0x00 (imu_x) + 0x00 (imu_y) + 0x00 (imu_z) + 0x00 (imu_yaw) + 0x00 (imu_pitch) + 0x00 (imu_roll) + 0x00 (radar_m) + 0x00 (ultrasonic_front_m) + 0x00 (ultrasonic_rear_m) + 0xB9 (bumper_front) + 0xB9 (bumper_rear) + 0x00 (gps_lat) + 0x00 (gps_lon) + 0xB9 (checksum)
# Format string only for the data (Exclude header, function code, and checksum):
format_string = '>BcHBbhhhhhhBBBccff'
data = incoming_data[2:-1]
print(f"Incoming data: {data.hex()}")
print(len(data))
print(len(incoming_data))
print(struct.calcsize(format_string))
if len(data) == struct.calcsize(format_string):
    print(f"Data length ({len(incoming_data)-3}) does not match format string size ({struct.calcsize(format_string)}).")
    # parsed_data = None
    # exclude first 2 bytes (header and function code) and last byte (checksum)
    data = incoming_data[2:-1]
    parsed_data = struct.unpack(format_string, data)
    print(parsed_data)
    parsed_dict = {
        "battery_%": parsed_data[0],
        "charging": 'Active' if parsed_data[1] else 'Inactive',
        "rpm": parsed_data[2] / 10.0,  # Divide by 10 as per description
        "brake_%": parsed_data[3],
        "steer_angle": parsed_data[4],
        "imu_x": parsed_data[5],
        "imu_y": parsed_data[6],
        "imu_z": parsed_data[7],
        "imu_yaw": parsed_data[8],
        "imu_pitch": parsed_data[9],
        "imu_roll": parsed_data[10],
        "radar_m": parsed_data[11] / 100.0,  # Divide by 100 as per description
        "ultrasonic_front_m": parsed_data[12] / 100.0,  # Divide by 100
        "ultrasonic_rear_m": parsed_data[13] / 100.0,  # Divide by 100
        "bumper_front": 'Active' if parsed_data[14] else 'Inactive',
        "bumper_rear": 'Active' if parsed_data[15] else 'Inactive',
        "gps_lat": parsed_data[16],
        "gps_lon": parsed_data[17]
    }
    print(parsed_dict)
else:
    print("Data length doesnt match format string size.")