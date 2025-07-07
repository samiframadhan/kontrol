import cv2
import cv2.aruco as aruco
import math
import numpy as np
import os
import zmq
import pyrealsense2 as rs
import json
import threading
from dataclasses import dataclass
from abc import ABC, abstractmethod

# -------------------------------------------------------------------
# --- 1. CONFIGURATION
# -------------------------------------------------------------------

@dataclass
class Config:
    """Holds all tunable parameters and constants."""
    # ZMQ
    ZMQ_SUB_URL: str = "tcp://localhost:5556"
    ZMQ_TOPIC: str = "teleop"

    # Camera and Video
    FRAME_WIDTH: int = 640
    FRAME_HEIGHT: int = 480
    FRAME_FPS: int = 30
    VIDEO_OUTPUT_FILE: str = 'acc_refactored.mp4'
    OPENCV_CAMERA_SOURCE: any = "40derajat77cmcrosstrack_run.avi"  # 0 for webcam, or "path/to/video.mp4"

    # Lane Detection
    LANE_HSV_LOWER: np.ndarray = np.array([0, 120, 120])
    LANE_HSV_UPPER: np.ndarray = np.array([40, 255, 255])
    PERSPECTIVE_WARP: float = 0.35
    MIN_CONTOUR_AREA: int = 100
    PIXELS_TO_METERS: float = 0.00725  # Conversion factor for distance

    # Stanley Controller
    STANLEY_GAIN: float = 1.0  # Proportional gain for cross-track error
    SPEED_EPSILON: float = 1e-6 # Small value to prevent division by zero

    LANE_DETECTION_SEGMENTS: int = 5  # Number of horizontal segments for lane detection

# -------------------------------------------------------------------
# --- 2. HARDWARE ABSTRACTION & UTILITIES
# -------------------------------------------------------------------

class Camera(ABC):
    """Abstract Base Class for camera sources."""
    @abstractmethod
    def start(self):
        """Initializes and starts the camera stream."""
        pass

    @abstractmethod
    def get_frame(self):
        """Returns a single frame (numpy array) from the camera."""
        pass

    @abstractmethod
    def stop(self):
        """Stops the camera stream and releases resources."""
        pass

class RealSenseCamera(Camera):
    """Camera implementation for Intel RealSense."""
    def __init__(self, config: Config):
        self.config = config
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.rs_config.enable_stream(
            rs.stream.color,
            config.FRAME_WIDTH,
            config.FRAME_HEIGHT,
            rs.format.bgr8,
            config.FRAME_FPS
        )

    def start(self):
        print("Starting RealSense camera...")
        self.pipeline.start(self.rs_config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def stop(self):
        print("Stopping RealSense camera.")
        self.pipeline.stop()

class OpenCVCamera(Camera):
    """Camera implementation for any source supported by OpenCV."""
    def __init__(self, config: Config, source):
        self.config = config
        self.source = source
        self.cap = None

    def start(self):
        print(f"Starting OpenCV camera with source: {self.source}...")
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            raise IOError(f"Cannot open OpenCV source: {self.source}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, self.config.FRAME_FPS)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def stop(self):
        if self.cap:
            print("Stopping OpenCV camera.")
            self.cap.release()

class ZmqSubscriber:
    """Manages ZMQ subscription in a separate thread for speed updates."""
    def __init__(self, config: Config):
        self.config = config
        self.speed = 0.0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(self.config.ZMQ_SUB_URL)
        socket.setsockopt_string(zmq.SUBSCRIBE, self.config.ZMQ_TOPIC)
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        print("ZMQ Subscriber thread started.")
        while not self._stop_event.is_set():
            socks = dict(poller.poll(timeout=100))
            if socket in socks and socks[socket] == zmq.POLLIN:
                _, data_json = socket.recv_multipart()
                data = json.loads(data_json)
                with self._lock:
                    self.speed = data.get('speed', 0.0)
        socket.close()
        context.term()
        print("ZMQ Subscriber thread stopped.")

    def start(self):
        self.thread.start()

    def stop(self):
        self._stop_event.set()
        self.thread.join()

    def get_speed(self):
        with self._lock:
            return self.speed

# (The StanleyController, LaneDetector, and Visualizer classes remain the same as the previous response)
# ...
class StanleyController:
    """Calculates steering angle using the Stanley control method."""
    @staticmethod
    def calculate_steering(cross_track_error, heading_error, speed, k, epsilon):
        cross_track_term = math.atan2(k * cross_track_error, speed + epsilon)
        return heading_error + cross_track_term

# ... (Config, Camera, ZmqSubscriber, and StanleyController classes are unchanged) ...

# -------------------------------------------------------------------
# --- 3. LANE DETECTION LOGIC (MODIFIED)
# -------------------------------------------------------------------

class LaneDetector:
    """Handles the full CV pipeline to detect lane and calculate errors."""
    def __init__(self, config: Config):
        self.config = config

    def _get_perspective_transform(self, img_shape):
        H, W = img_shape
        warp = self.config.PERSPECTIVE_WARP
        mirror_point = 1 - warp
        src = np.float32([[W * warp, H], [0, 0], [W, 0], [W * mirror_point, H]])
        dst = np.float32([[0, H], [0, 0], [W, 0], [W, H]])
        return cv2.getPerspectiveTransform(dst, src)
    
    @staticmethod
    def _region_of_interest(img, vertices):
        """Applies a mask to an image based on the provided vertices."""
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, vertices, 255)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    @staticmethod
    def _find_lane_contours(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        centers = []
        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > Config.MIN_CONTOUR_AREA:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append({'x': cx, 'y': cy, 'area': M['m00']})
        return contours, centers

    def _calculate_perpendicular_error(self, m, b, H, W):
        """Calculates the precise perpendicular distance from the vehicle to the lane line."""
        bottom_center = (W // 2, H)
        
        # Handle the case of a horizontal line to avoid division by zero for perp_m
        if abs(m) < 1e-6:
            x_int = bottom_center[0]
            y_int = b
        else:
            # Calculate the slope and intercept of the perpendicular line
            perp_m = -1 / m
            perp_b = bottom_center[1] - perp_m * bottom_center[0]
            
            # Calculate the intersection point of the two lines
            x_int = (perp_b - b) / (m - perp_m)
            y_int = m * x_int + b
            
        intersection_point = (int(x_int), int(y_int))
        
        # Calculate the magnitude of the error (distance)
        error_pixels = math.hypot(x_int - bottom_center[0], y_int - bottom_center[1])
        
        # Determine the sign of the error based on the lane's position
        x_at_bottom = (H - b) / m
        if x_at_bottom < W // 2:
            error_pixels = -error_pixels
            
        return error_pixels, intersection_point

    def process_frame(self, img):
        """Main pipeline to process a single frame and return lane data."""
        H, W = img.shape[:2]
        M = self._get_perspective_transform((H, W))
        img_warped = cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_LINEAR)

        hsv_img = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv_img, self.config.LANE_HSV_LOWER, self.config.LANE_HSV_UPPER)
        
        all_contours = []
        all_centers = []
        region_height = H // self.config.LANE_DETECTION_SEGMENTS

        for i in range(self.config.LANE_DETECTION_SEGMENTS):
            # Define the vertices for the current horizontal segment
            segment_vertices = np.array([[
                (0, H - (i + 1) * region_height),
                (W, H - (i + 1) * region_height),
                (W, H - i * region_height),
                (0, H - i * region_height),
            ]], dtype=np.int32)

            # Mask the yellow-filtered image to this specific segment
            segment_mask = self._region_of_interest(mask_yellow, segment_vertices)

            # Find contours within this isolated segment
            contours, centers = self._find_lane_contours(segment_mask)
            all_contours.extend(contours)
            all_centers.extend(centers)

        centers_sorted = sorted(all_centers, key=lambda c: -c['y'])

        pipeline_data = {
            'warped_image': img_warped, 'color_mask': mask_yellow, 'contours': all_contours,
            'centers': all_centers, 'cross_track_error': 0.0, 'heading_error': 0.0,
            'lane_points': [], 'cte_intersection_point': None
        }

        if len(centers_sorted) >= 2:
            p1 = (centers_sorted[0]['x'], centers_sorted[0]['y'])
            p2 = (centers_sorted[1]['x'], centers_sorted[1]['y'])
            pipeline_data['lane_points'] = [p1, p2]

            dx, dy = p2[0] - p1[0], p2[1] - p1[1]

            if abs(dx) > 1e-6:
                m = dy / dx
                b = p1[1] - m * p1[0]

                pipeline_data['heading_error'] = (math.pi / 2) - math.atan2(-dy, dx)

                error_pixels, intersection_point = self._calculate_perpendicular_error(m, b, H, W)
                pipeline_data['cross_track_error'] = error_pixels * self.config.PIXELS_TO_METERS
                pipeline_data['cte_intersection_point'] = intersection_point

        return pipeline_data

# -------------------------------------------------------------------
# --- 4. VISUALIZATION (MODIFIED)
# -------------------------------------------------------------------

class Visualizer:
    """Handles all OpenCV drawing operations."""
    def __init__(self, config: Config):
        self.config = config

    def draw_pipeline_data(self, image, data, steering_angle):
        H, W = image.shape[:2]
        
        cv2.drawContours(image, data['contours'], -1, (255, 0, 255), 1)
        for center in data['centers']:
            cv2.circle(image, (center['x'], center['y']), 5, (0, 255, 0), -1)

        if data['lane_points']:
            cv2.line(image, data['lane_points'][0], data['lane_points'][1], (255, 0, 0), 3)

        # Draw the perpendicular cross-track error line if it was calculated
        if data['cte_intersection_point']:
            bottom_center = (W // 2, H)
            cv2.line(image, bottom_center, data['cte_intersection_point'], (255, 0, 255), 2)

        # Draw steering angle arrow
        arrow_length = 100
        x0, y0 = W // 2, H
        arrow_angle = (math.pi / 2) - steering_angle
        x1 = int(x0 + arrow_length * math.cos(arrow_angle))
        y1 = int(y0 - arrow_length * math.sin(arrow_angle))
        cv2.arrowedLine(image, (x0, y0), (x1, y1), (0, 0, 255), 3, tipLength=0.2)
        
        # Display text info
        cv2.putText(image, f"CTE: {data['cross_track_error']:.3f} m", (10, H - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"HE: {math.degrees(data['heading_error']):.1f} deg", (10, H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"Steer: {math.degrees(steering_angle):.1f} deg", (10, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        return image

# -------------------------------------------------------------------
# --- 5. MAIN EXECUTION
# -------------------------------------------------------------------

def main():
    config = Config()

    # --- CHOOSE YOUR CAMERA SOURCE HERE ---
    # To use a webcam or video file:
    camera = OpenCVCamera(config, source=config.OPENCV_CAMERA_SOURCE)
    # To use an Intel RealSense camera:
    # camera = RealSenseCamera(config)
    # ------------------------------------

    # --- Initialization ---
    speed_subscriber = ZmqSubscriber(config)
    lane_detector = LaneDetector(config)
    visualizer = Visualizer(config)
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    result_writer = cv2.VideoWriter(
        config.VIDEO_OUTPUT_FILE, fourcc, config.FRAME_FPS,
        (config.FRAME_WIDTH * 2, config.FRAME_HEIGHT)
    )

    try:
        camera.start()
        speed_subscriber.start()
        print("Starting main loop. Press 'q' to quit.")

        while True:
            # 1. Get Frame
            frame = camera.get_frame()
            if frame is None:
                print("End of video stream or camera error.")
                break

            # 2. Process for Lane Data
            lane_data = lane_detector.process_frame(frame)
            
            # 3. Get Current Speed
            current_speed = speed_subscriber.get_speed()
            current_speed = 1.0

            # 4. Calculate Control Signal
            steering_angle = StanleyController.calculate_steering(
                lane_data['cross_track_error'],
                lane_data['heading_error'],
                current_speed,
                config.STANLEY_GAIN,
                config.SPEED_EPSILON
            )
            
            # 5. Visualize Results
            warped_view = cv2.cvtColor(lane_data['color_mask'], cv2.COLOR_GRAY2BGR)
            annotated_warped = visualizer.draw_pipeline_data(warped_view, lane_data, steering_angle)
            
            # 6. Display and Save
            stacked_output = np.hstack((frame, annotated_warped))
            cv2.imshow('Lane Following View', stacked_output)
            result_writer.write(stacked_output)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred in the main loop: {e}")
    finally:
        # --- Cleanup ---
        print("Shutting down...")
        camera.stop()
        speed_subscriber.stop()
        result_writer.release()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()