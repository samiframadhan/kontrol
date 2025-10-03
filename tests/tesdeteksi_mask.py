# process_video.py (updated): robust previous-vs-current paired contour matching by midpoint proximity
import cv2
import logging
import time
import numpy as np
import argparse
import os
from typing import List, Tuple, Optional, Set

# ---------------------- Logging ----------------------
def setup_logging():
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S')

def on_trackbar(val):
    pass  # Placeholder for trackbar callback

# ---------------------- Main pipeline ----------------------
def main():
    setup_logging()
    logger = logging.getLogger("VideoProcessor")

    parser = argparse.ArgumentParser(description="Process a video file for lane detection.")
    parser.add_argument("input_file", type=str, help="Path to the input video file.")
    args = parser.parse_args()

    if not os.path.exists(args.input_file):
        logger.error(f"Input file not found: {args.input_file}")
        return

    cap = cv2.VideoCapture(args.input_file)
    if not cap.isOpened():
        logger.error(f"Could not open video file: {args.input_file}")
        return

    frame_width, frame_height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    datetime_str = time.strftime("%Y%m%d-%H%M%S")
    output_filename = f'output_{datetime_str}.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))
    logger.info(f"Processing '{args.input_file}'. Output will be saved to '{output_filename}'.")

    try:
        calibration_file = '../calibrationimx708hdr.yaml'
        calibration_fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
        camera_matrix, dist_coeffs = calibration_fs.getNode('camera_matrix').mat(), calibration_fs.getNode('distortion_coefficients').mat()
        calibration_fs.release()
    except Exception as e:
        logger.error(f"Calibration file '{calibration_file}' not found or invalid. Error: {e}. Cannot proceed.")
        cap.release()
        return

    previous_paired = None

    ## MOD-HSV: State variables for pausing and interactive HSV thresholding
    is_paused = False
    sliders_created = False
    last_frame = None
    window_name = "Lane Detection Viewer"
    # Initial HSV values for white lanes. H(0-179), S(0-255), V(0-255) in OpenCV
    h_min, s_min, v_min = 0, 0, 214
    h_max, s_max, v_max = 179, 24, 255

    cv2.namedWindow(window_name)

    try:
        while cap.isOpened():
            if not is_paused:
                ret, frame = cap.read()
                if not ret:
                    logger.info("End of video file reached.")
                    break
                last_frame = frame.copy()

            if last_frame is None:
                continue

            frame_to_process = cv2.undistort(last_frame, camera_matrix, dist_coeffs)
            
            ## MOD-HSV: Convert to HSV color space
            hsv = cv2.cvtColor(frame_to_process, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

            ## MOD-HSV: If paused, get current values from all 6 sliders
            if is_paused and sliders_created:
                h_min = cv2.getTrackbarPos('H Min', window_name)
                h_max = cv2.getTrackbarPos('H Max', window_name)
                s_min = cv2.getTrackbarPos('S Min', window_name)
                s_max = cv2.getTrackbarPos('S Max', window_name)
                v_min = cv2.getTrackbarPos('V Min', window_name)
                v_max = cv2.getTrackbarPos('V Max', window_name)

            ## MOD-HSV: Use the dynamic HSV threshold values
            lower_bound = np.array([h_min, s_min, v_min])
            upper_bound = np.array([h_max, s_max, v_max])
            white_mask = cv2.inRange(hsv, lower_bound, upper_bound)
            
            kernel = np.ones((3, 3), np.uint8)
            white_mask = cv2.erode(white_mask, kernel, iterations=2)
            
            writer.write(last_frame)

            # Viewer
            display_frame = frame_to_process.copy()
            ## MOD-HSV: Display current HSV threshold values on screen
            text_h = f"H: {h_min}-{h_max}"
            text_sv = f"S: {s_min}-{s_max}  V: {v_min}-{v_max}"
            cv2.putText(display_frame, "Lane Detection", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(display_frame, text_h, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_frame, text_sv, (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            combined_frame = np.hstack([display_frame, cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)])
            cv2.imshow(window_name, combined_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logger.info("'q' pressed, shutting down.")
                break
            elif key == ord(' '):
                is_paused = not is_paused
                if is_paused:
                    logger.info("Video paused. Adjust HSV sliders. Press space to resume.")
                    ## MOD-HSV: Create 6 sliders only once when first paused
                    if not sliders_created:
                        # Note: Hue max is 179 in OpenCV
                        cv2.createTrackbar('H Min', window_name, h_min, 179, on_trackbar)
                        cv2.createTrackbar('H Max', window_name, h_max, 179, on_trackbar)
                        cv2.createTrackbar('S Min', window_name, s_min, 255, on_trackbar)
                        cv2.createTrackbar('S Max', window_name, s_max, 255, on_trackbar)
                        cv2.createTrackbar('V Min', window_name, v_min, 255, on_trackbar)
                        cv2.createTrackbar('V Max', window_name, v_max, 255, on_trackbar)
                        sliders_created = True
                else:
                    logger.info("Video resumed.")

    except KeyboardInterrupt:
        logger.info("Interrupted by user. Shutting down.")
    finally:
        logger.info("Releasing resources...")
        cap.release()
        writer.release()
        cv2.destroyAllWindows()
        logger.info("Processing finished successfully.")

if __name__ == "__main__":
    main()