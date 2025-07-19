import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import argparse
from abc import ABC, abstractmethod

# --- Constants ---
# Path to the calibration file
CALIBRATION_FILE = "calibrationd415.yaml"
# Physical size of the ArUco marker in meters
MARKER_SIZE = 0.05  # 5 centimeters

# -------------------------------------------------------------------
# --- 1. CAMERA ABSTRACTION CLASSES ---------------------------------
# -------------------------------------------------------------------

class Camera(ABC):
    """Abstract base class for a generic camera."""
    @abstractmethod
    def start(self):
        """Starts the camera stream."""
        pass

    @abstractmethod
    def stop(self):
        """Stops the camera stream."""
        pass

    @abstractmethod
    def get_frames(self):
        """
        Retrieves frames from the camera.

        Returns:
            A tuple containing (color_image, depth_frame).
            For cameras without depth, depth_frame will be None.
        """
        pass

class RealSenseCamera(Camera):
    """Camera implementation for Intel RealSense."""
    def __init__(self, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.align = rs.align(rs.stream.color)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.result_writer = cv2.VideoWriter(
            "res.mp4", fourcc, 30,
            (640,480)
        )

    def start(self):
        print("Starting RealSense camera...")
        self.pipeline.start(self.config)

    def stop(self):
        print("Stopping RealSense camera.")
        self.pipeline.stop()
        self.result_writer.release()

    def get_frames(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return None, None
            
            color_image = np.asanyarray(color_frame.get_data())
            self.result_writer.write(color_image)
            return color_image, depth_frame
        except RuntimeError as e:
            print(f"Error getting frames from RealSense: {e}")
            return None, None

class OpenCVCamera(Camera):
    """Camera implementation for any source supported by OpenCV."""
    def __init__(self, source=0, width=640, height=480, fps=30):
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None

    def start(self):
        print(f"Starting OpenCV camera with source: {self.source}...")
        self.cap = cv2.VideoCapture(self.source)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 2)  # Disable auto exposure
        # self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)  # Disable auto white balance
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.result_writer = cv2.VideoWriter(
            "res.mp4", fourcc, 30,
            (640,480)
        )
        if not self.cap.isOpened():
            raise IOError(f"Cannot open OpenCV source: {self.source}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

    def stop(self):
        if self.cap:
            self.result_writer.release()
            print("Stopping OpenCV camera.")
            self.cap.release()

    def get_frames(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Return the color frame and None for the depth frame
                self.result_writer.write(frame)
                return frame, None
        return None, None

# -------------------------------------------------------------------
# --- 2. MAIN APPLICATION LOGIC -------------------------------------
# -------------------------------------------------------------------

def main():
    """
    Main function to run the ArUco marker distance estimation.
    """
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(description="ArUco Marker Pose and Distance Estimation.")
    parser.add_argument("--camera", type=str, default="realsense", choices=["realsense", "opencv"],
                        help="The type of camera to use.")
    parser.add_argument("--source", type=str, default="0",
                        help="The source for the OpenCV camera (e.g., '0' for webcam, or a path to a video file).")
    args = parser.parse_args()


    # --- Load Camera Calibration ---
    try:
        fs = cv2.FileStorage(CALIBRATION_FILE, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            print(f"Error: Could not open calibration file: {CALIBRATION_FILE}")
            return
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeff").mat()
        fs.release()
    except Exception as e:
        print(f"Error loading calibration file: {e}")
        return

    if camera_matrix is None or dist_coeffs is None:
        print("Error: Camera matrix or distortion coefficients not found in calibration file.")
        return

    # --- Initialize Selected Camera ---
    if args.camera == "realsense":
        camera = RealSenseCamera()
    else: # args.camera == "opencv"
        source = int(args.source) if args.source.isdigit() else args.source
        camera = OpenCVCamera(source=source)

    try:
        camera.start()
    except Exception as e:
        print(f"Error starting camera: {e}")
        return

    # --- ArUco Detection Setup ---
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    print("Streaming started. Press 'q' to quit.")

    try:
        while True:
            # Get frames from the selected camera
            color_image, depth_frame = camera.get_frames()

            if color_image is None:
                print("End of stream or camera disconnected.")
                break

            # --- ArUco Marker Detection ---
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                # Estimate pose of each marker
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, camera_matrix, dist_coeffs)
                aruco.drawDetectedMarkers(color_image, corners, ids)

                for i, marker_id in enumerate(ids):
                    # Draw axis for each marker
                    cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.025)

                    # --- Get Distance (conditionally) ---
                    display_text = f"ID: {marker_id[0]}"
                    # Only calculate distance if a depth frame is available (i.e., from RealSense)
                    if depth_frame is not None:
                        marker_corners = corners[i][0]
                        center_x = int(np.mean(marker_corners[:, 0]))
                        center_y = int(np.mean(marker_corners[:, 1]))

                        # Ensure the center point is within frame bounds
                        h, w, _ = color_image.shape
                        if 0 <= center_x < w and 0 <= center_y < h:
                            distance = depth_frame.get_distance(center_x, center_y)
                            if distance > 0:
                                display_text += f" Dist: {distance:.2f} m"

                    # --- Display Information ---
                    text_position = (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 15)
                    cv2.putText(color_image, display_text, text_position,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

            # Display the resulting frame
            cv2.imshow('ArUco Detection', color_image)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Stop streaming and clean up
        camera.stop()
        cv2.destroyAllWindows()
        print("Streaming stopped.")

if __name__ == '__main__':
    main()