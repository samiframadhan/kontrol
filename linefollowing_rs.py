# linefollowing_rs_yaml.py (Integrated Version with YAML Config, Improved Logging, and ZMQ Frame Subscription)
import cv2
import math
import numpy as np
import zmq
import time
import logging
import yaml # Import the YAML library
from pyrealsense2 import pyrealsense2 as rs
from dataclasses import dataclass
from abc import ABC, abstractmethod
import steering_command_pb2

# -------------------------------------------------------------------
# --- 1. CONFIGURATION
# -------------------------------------------------------------------

def load_config(config_path='linedetection.yaml'):
    """
    Loads configuration from a YAML file and processes values.

    To use the new ZMQ camera source, update your YAML file as follows:

    camera:
      source_type: 'zmq'  # Can be 'realsense', 'opencv', or 'zmq'
      # ... other camera params like frame_width, frame_height, etc. are still needed

    zmq:
      # For publishing steering commands (no change)
      pub_steer_url: "tcp://*:5557"
      steer_topic: "steer_cmd"

      # Add these for subscribing to frames
      sub_frame_url: "tcp://localhost:5556" # URL of the frame publisher
      frame_topic: "cam_frame"               # Topic to subscribe to for frames
    """
    logging.info(f"Loading configuration from {config_path}...")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Convert HSV lists from YAML to NumPy arrays and Tuples for OpenCV
    ld_config = config['lane_detection']
    ld_config['hsv_lower_np'] = np.array(ld_config['hsv_lower'])
    ld_config['hsv_upper_np'] = np.array(ld_config['hsv_upper'])
    ld_config['hsv_lower_gpu_tuple'] = tuple(ld_config['hsv_lower'])
    ld_config['hsv_upper_gpu_tuple'] = tuple(ld_config['hsv_upper'])

    logging.info("Configuration loaded successfully.")
    return config

class KeyboardInput():
    """Handles keyboard input for controlling the application."""
    def __init__(self):
        self.UNIX = False
        self.tty = None
        self.termios = None
        self.select = None
        self.settings = None
        self.sys = None
        try:
            import sys
            if sys.stdin.isatty():
                import tty, termios, select
                self.sys = sys
                self.UNIX = True
                self.tty, self.termios, self.select = tty, termios, select
                self.settings = termios.tcgetattr(sys.stdin)
        except (ImportError, AttributeError):
             pass

    def get_key(self, timeout=0.001):
        """Gets a single character from standard input."""
        if self.UNIX and self.settings:
            self.tty.setraw(self.sys.stdin.fileno())
            rlist, _, _ = self.select.select([self.sys.stdin], [], [], timeout)
            if rlist:
                key = self.sys.stdin.read(1)
            else:
                key = ''
            self.termios.tcsetattr(self.sys.stdin, self.termios.TCSADRAIN, self.settings)
            return key
        return None

def setup_logging():
    """Configures the logging for the application."""
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# -------------------------------------------------------------------
# --- 2. CORE CLASSES
# -------------------------------------------------------------------

class Camera(ABC):
    @abstractmethod
    def start(self): pass
    @abstractmethod
    def get_frame(self): pass
    @abstractmethod
    def stop(self): pass

class RealSenseCamera(Camera):
    """Camera implementation for Intel RealSense."""
    def __init__(self, cam_config: dict):
        self.config = cam_config
        self.pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.rs_config.enable_stream(rs.stream.color, self.config['frame_width'], self.config['frame_height'], rs.format.bgr8, self.config['frame_fps'])
    def start(self):
        logging.info("Starting RealSense camera...")
        self.pipeline.start(self.rs_config)
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            logging.warning("No color frame received from RealSense camera.")
            return None
        return np.asanyarray(color_frame.get_data())
    def stop(self):
        logging.info("Stopping RealSense camera.")
        self.pipeline.stop()

class OpenCVCamera(Camera):
    """Camera implementation for any source supported by OpenCV."""
    def __init__(self, cam_config: dict, source):
        self.config = cam_config; self.source = source; self.cap = None
    def start(self):
        logging.info(f"Starting OpenCV camera with source: {self.source}...")
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened(): raise IOError(f"Cannot open OpenCV source: {self.source}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['frame_width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['frame_height'])
        self.cap.set(cv2.CAP_PROP_FPS, self.config['frame_fps'])
    def get_frame(self):
        ret, frame = self.cap.read()
        return frame if ret else None
    def stop(self):
        if self.cap:
            logging.info("Stopping OpenCV camera.")
            self.cap.release()

class ZMQCamera(Camera):
    """Camera implementation for a ZMQ subscription source."""
    def __init__(self, cam_config: dict, zmq_sub_config: dict, context: zmq.Context):
        self.config = cam_config
        self.zmq_config = zmq_sub_config
        self.context = context # Use a shared ZMQ context
        self.socket = None
        self.poller = None
        self.frame_shape = (self.config['frame_height'], self.config['frame_width'], 3)
        self.dtype = np.uint8

    def start(self):
        logging.info(f"Connecting to ZMQ frame publisher at {self.zmq_config['sub_frame_url']}...")
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.zmq_config['sub_frame_url'])
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self.zmq_config['sub_frame_topic'])
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        logging.info(f"Subscribed to frame topic: '{self.zmq_config['sub_frame_topic']}'")

    def get_frame(self):
        # Poll the socket for a message with a timeout of 0 to return immediately
        socks = dict(self.poller.poll(0))
        if self.socket in socks and socks[self.socket] == zmq.POLLIN:
            try:
                # Receive topic and frame data
                topic = self.socket.recv_string()
                frame_bytes = self.socket.recv()
                frame = np.frombuffer(frame_bytes, dtype=self.dtype)
                return frame.reshape(self.frame_shape)
            except Exception as e:
                logging.error(f"Error processing ZMQ frame: {e}")
                return None
        return None # No message received

    def stop(self):
        if self.socket:
            logging.info("Closing ZMQ frame subscriber socket.")
            self.socket.close()
        # The shared context is terminated in the main function's finally block

class StanleyController:
    """Calculates steering angle using the Stanley control method."""
    @staticmethod
    def calculate_steering(cross_track_error, heading_error, speed, k, epsilon):
        cross_track_term = math.atan2(k * cross_track_error, speed + epsilon)
        return heading_error + cross_track_term

class LaneDetector:
    """Handles the full CV pipeline to detect lane and calculate errors using GPU."""
    def __init__(self, config: dict, use_cuda: bool):
        self.config = config
        self.ld_config = config['lane_detection'] # Shortcut to lane detection params
        self.use_cuda = use_cuda
        if self.use_cuda:
            self.gpu_frame = cv2.cuda_GpuMat()
            self.gpu_warped = cv2.cuda_GpuMat()
            self.gpu_hsv = cv2.cuda_GpuMat()
            self.gpu_mask = cv2.cuda_GpuMat()
            logging.info("LaneDetector initialized with CUDA support.")
        else:
            logging.info("LaneDetector initialized for CPU processing.")

    def _get_perspective_transform(self, img_shape):
        H, W = img_shape
        warp = self.ld_config['perspective_warp']
        mirror_point = 1 - warp
        src = np.float32([[W * warp, H], [0, 0], [W, 0], [W * mirror_point, H]])
        dst = np.float32([[0, H], [0, 0], [W, 0], [W, H]])
        return cv2.getPerspectiveTransform(dst, src)

    def _find_lane_contours(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        centers = []
        min_area = self.ld_config['min_contour_area']
        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > min_area:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append({'x': cx, 'y': cy, 'area': M['m00']})
        return contours, centers

    @staticmethod
    def _region_of_interest(img, vertices):
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, vertices, 255)
        return cv2.bitwise_and(img, mask)

    def _calculate_perpendicular_error(self, m, b, H, W):
        bottom_center = (W // 2, H)
        if abs(m) < 1e-6:
            x_int, y_int = bottom_center[0], b
        else:
            perp_m = -1 / m
            perp_b = bottom_center[1] - perp_m * bottom_center[0]
            x_int = (perp_b - b) / (m - perp_m)
            y_int = m * x_int + b
        intersection_point = (int(x_int), int(y_int))
        error_pixels = math.hypot(x_int - bottom_center[0], y_int - bottom_center[1])
        x_at_bottom = (H - b) / m if abs(m) > 1e-6 else float('inf')
        if x_at_bottom < W // 2: error_pixels = -error_pixels
        return error_pixels, intersection_point

    def process_frame(self, img):
        H, W = img.shape[:2]
        M = self._get_perspective_transform((H, W))

        if self.use_cuda:
            self.gpu_frame.upload(img)
            self.gpu_warped = cv2.cuda.warpPerspective(self.gpu_frame, M, (W, H), flags=cv2.INTER_LINEAR)
            self.gpu_hsv = cv2.cuda.cvtColor(self.gpu_warped, cv2.COLOR_BGR2HSV)
            self.gpu_mask = cv2.cuda.inRange(self.gpu_hsv, self.ld_config['hsv_lower_gpu_tuple'], self.ld_config['hsv_upper_gpu_tuple'])
            mask_yellow = self.gpu_mask.download()
            img_warped = self.gpu_warped.download()
        else:
            img_warped = cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_LINEAR)
            hsv_img = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(hsv_img, self.ld_config['hsv_lower_np'], self.ld_config['hsv_upper_np'])

        all_contours, all_centers = [], []
        num_segments = self.ld_config['segments']
        region_height = H // num_segments
        for i in range(num_segments):
            segment_vertices = np.array([[(0, H - (i + 1) * region_height), (W, H - (i + 1) * region_height),
                                          (W, H - i * region_height), (0, H - i * region_height)]], dtype=np.int32)
            segment_mask = self._region_of_interest(mask_yellow, segment_vertices)
            contours, centers = self._find_lane_contours(segment_mask)
            all_contours.extend(contours)
            all_centers.extend(centers)

        centers_sorted = sorted(all_centers, key=lambda c: -c['y'])
        pipeline_data = {'warped_image': img_warped, 'color_mask': mask_yellow, 'contours': all_contours,
                         'centers': all_centers, 'cross_track_error': 0.0, 'heading_error': 0.0,
                         'lane_points': [], 'cte_intersection_point': None}

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
                pipeline_data['cross_track_error'] = error_pixels * self.ld_config['pixels_to_meters']
                pipeline_data['cte_intersection_point'] = intersection_point
        else:
            logging.debug("Not enough lane points detected to calculate errors.")

        return pipeline_data

class Visualizer:
    """Handles all OpenCV drawing operations."""
    def draw_pipeline_data(self, image, data, steering_angle):
        H, W = image.shape[:2]
        cv2.drawContours(image, data['contours'], -1, (255, 0, 255), 1)
        for center in data['centers']: cv2.circle(image, (center['x'], center['y']), 5, (0, 255, 0), -1)
        if data['lane_points']: cv2.line(image, data['lane_points'][0], data['lane_points'][1], (255, 0, 0), 3)
        if data['cte_intersection_point']:
            bottom_center = (W // 2, H); cv2.line(image, bottom_center, data['cte_intersection_point'], (255, 0, 255), 2)
        arrow_length = 100; x0, y0 = W // 2, H
        arrow_angle = (math.pi / 2) - steering_angle
        x1, y1 = int(x0 + arrow_length * math.cos(arrow_angle)), int(y0 - arrow_length * math.sin(arrow_angle))
        cv2.arrowedLine(image, (x0, y0), (x1, y1), (0, 0, 255), 3, tipLength=0.2)
        cv2.putText(image, f"CTE: {data['cross_track_error']:.3f} m", (10, H - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"HE: {math.degrees(data['heading_error']):.1f} deg", (10, H - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(image, f"Steer: {math.degrees(steering_angle):.1f} deg", (10, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        return image

# -------------------------------------------------------------------
# --- 3. MAIN EXECUTION
# -------------------------------------------------------------------

def main():
    setup_logging()
    config = load_config()
    keyboard = KeyboardInput()

    use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
    logging.info(f"CUDA Available: {use_cuda}. {'Using GPU.' if use_cuda else 'Running on CPU.'}")

    # Create a single ZMQ context for the entire application
    context = zmq.Context()

    # --- Camera and ZMQ Setup ---
    cam_config = config['camera']
    zmq_config = config['zmq']

    camera = None
    source_type = cam_config.get('source_type', '').lower()

    if source_type == 'realsense':
        camera = RealSenseCamera(cam_config)
    elif source_type == 'opencv':
        camera = OpenCVCamera(cam_config, source=cam_config.get('opencv_source'))
    elif source_type == 'zmq':
        # Pass the shared context to the ZMQCamera
        camera = ZMQCamera(cam_config, zmq_config, context)
    else:
        raise ValueError(f"Invalid or missing camera source_type in config: {source_type}")

    # Setup steering command publisher using the shared context
    pub_socket = context.socket(zmq.PUB)
    pub_socket.bind(zmq_config['pub_steer_url'])
    logging.info(f"Publishing lane assist angles on {zmq_config['pub_steer_url']}")

    lane_detector = LaneDetector(config, use_cuda)
    visualizer = Visualizer()

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    result_writer = cv2.VideoWriter(
        cam_config['video_output_file'], fourcc, cam_config['frame_fps'],
        (cam_config['frame_width'] * 2, cam_config['frame_height'])
    )
    logging.info(f"Video output will be saved to {cam_config['video_output_file']}")
    count = 0

    try:
        camera.start()
        logging.info("Starting main loop. Press 'q' to quit.")
        start_time = time.time()

        while True:
            frame = camera.get_frame()
            if frame is None:
                # For ZMQ, this just means no message arrived; for others, it's end of stream
                if source_type != 'zmq':
                    logging.info("End of video stream or camera error.")
                    break
                # If ZMQ, just wait for the next frame without erroring out
                key = keyboard.get_key()
                if key == 'q':
                    logging.info("'q' pressed in console, shutting down.")
                    break
                time.sleep(0.01) # Avoid busy-waiting
                continue

            lane_data = lane_detector.process_frame(frame)

            sc_config = config['stanley_controller']
            steering_angle_rad = StanleyController.calculate_steering(
                lane_data['cross_track_error'],
                lane_data['heading_error'],
                sc_config['assumed_speed_for_calc'],
                sc_config['gain'],
                sc_config['speed_epsilon']
            )
            steering_angle_deg = math.degrees(steering_angle_rad)
            count += 1

            command = steering_command_pb2.SteeringCommand()
            command.auto_steer_angle = steering_angle_deg
            serialized_command = command.SerializeToString()
            pub_socket.send_string(zmq_config['steer_topic'], flags=zmq.SNDMORE)
            pub_socket.send(serialized_command)
            logging.debug(f"Published steering angle: {steering_angle_deg:.2f} degrees")

            warped_view = lane_data.get('warped_image', np.zeros_like(frame))
            annotated_warped = visualizer.draw_pipeline_data(warped_view, lane_data, steering_angle_rad)

            stacked_output = np.hstack((frame, annotated_warped))
            if cam_config['use_display']:
                cv2.imshow('Lane Following View', stacked_output)

            # Check for 'q' key press in display window OR console
            key = keyboard.get_key()
            if (cam_config['use_display'] and cv2.waitKey(1) & 0xFF == ord('q')) or key == 'q':
                logging.info("'q' pressed, shutting down.")
                break

            result_writer.write(stacked_output)

            if count > 0 and count % 30 == 0:
                elapsed_time = time.time() - start_time
                fps = count / elapsed_time
                logging.info(f"Current Processing FPS: {fps:.2f}")

    except Exception as e:
        logging.exception("An unhandled error occurred in the main loop:")
    finally:
        print("\n")
        logging.info("Shutting down...")
        if camera:
            camera.stop()
        result_writer.release()
        cv2.destroyAllWindows()
        pub_socket.close()
        context.term() # Terminate the single, shared context
        logging.info("Shutdown complete.")

if __name__ == "__main__":
    main()
