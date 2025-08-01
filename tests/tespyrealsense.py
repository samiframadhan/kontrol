import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()
for device in devices:
    device_name = device.get_info(rs.camera_info.serial_number)

    print(f"Device: {device.get_info(rs.camera_info.name)}, Serial: {device.get_info(rs.camera_info.serial_number)}")