import pyrealsense2 as rs
import cv2
import numpy as np

def main(output_file='realsense.mp4', duration=10, fps=30):
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    # Disable auto white balance
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)

    # Start streaming
    profile = pipeline.start(config)

    # profile.get_device().first_color_sensor().set_option(rs.option.enable_auto_white_balance, 0)  # Disable auto white balance
    # Video writer setup
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    out = cv2.VideoWriter(output_file, fourcc, fps, (640, 480))

    print("Recording... Press 'q' to stop early.")
    frame_count = 0
    max_frames = duration * fps

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            out.write(color_image)
            cv2.imshow('RealSense', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Stop streaming
        pipeline.stop()
        out.release()
        cv2.destroyAllWindows()
        print("Recording finished.")

if __name__ == "__main__":
    main()