import cv2
import cv2.aruco as aruco
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import pyrealsense2 as rs

class CameraCalibration:
    def __init__(self, yaml_path):
        self.camera_matrix, self.dist_coeffs = self.load_calibration(yaml_path)

    @staticmethod
    def load_calibration(yaml_path):
        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(f"Calibration file '{yaml_path}' does not exist.")
        fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            fs.release()
            raise IOError(f"Failed to open calibration file '{yaml_path}'.")
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("distortion_coefficients").mat()
        fs.release()
        return camera_matrix, dist_coeffs

class ArucoDetector:
    def __init__(self, calibration: CameraCalibration):
        self.camera_matrix = calibration.camera_matrix
        self.dist_coeffs = calibration.dist_coeffs
        self.ARUCO_PARAMETERS = aruco.DetectorParameters()
        self.ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.axis = np.float32([[-.5,-.5,0], [-.5,.5,0], [.5,.5,0], [.5,-.5,0],
                                [-.5,-.5,1],[-.5,.5,1],[.5,.5,1],[.5,-.5,1] ])

    def detect(self, img):
        board = aruco.GridBoard(
            size=(1, 1),
            markerLength=0.1,
            markerSeparation=0.01,
            dictionary=self.ARUCO_DICT)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.ARUCO_DICT, parameters=self.ARUCO_PARAMETERS)
        rotation_vectors, translation_vectors = [], []
        if ids is not None:
            corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
                gray, board, corners, ids, rejectedImgPoints)
            for i in range(len(ids)):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.1, self.camera_matrix, self.dist_coeffs)
                rotation_vectors.append(rvec)
                translation_vectors.append(tvec)
            img = aruco.drawDetectedMarkers(img, corners, ids)
        return img, corners, ids

class LaneFollower:
    MIN_AREA = 100
    MIN_AREA_TRACK = 300

    def __init__(self, calibration: CameraCalibration =None, aruco_detector: ArucoDetector =None):
        self.calibration = calibration
        self.aruco_detector = aruco_detector
        self.detect_aruco = True
        if self.calibration is None and self.aruco_detector is None:
            self.detect_aruco = False

    @staticmethod
    def grayscale(img):
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    @staticmethod
    def canny(img, low_threshold, high_threshold):
        return cv2.Canny(img, low_threshold, high_threshold)

    @staticmethod
    def gaussian_blur(img, kernel_size):
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    @staticmethod
    def crop_size(height, width):
        return (1*height//3, height, width//4, 3*width//4)

    @staticmethod
    def get_contour_data(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mark = {}
        line = {}
        for contour in contours:
            M = cv2.moments(contour)
            if M['m00'] > LaneFollower.MIN_AREA:
                if (M['m00'] > LaneFollower.MIN_AREA_TRACK):
                    line['x'] = int(M["m10"]/M["m00"])
                    line['y'] = int(M["m01"]/M["m00"])
                else:
                    if (not mark) or (mark['y'] > int(M["m01"]/M["m00"])):
                        mark['y'] = int(M["m01"]/M["m00"])
                        mark['x'] = int(M["m10"]/M["m00"])
        if mark and line:
            mark_side = "right" if mark['x'] > line['x'] else "left"
        else:
            mark_side = None
        return (line, mark_side, contours)

    @staticmethod
    def draw_contours(mask, contours):
        for contour in contours:
            cv2.drawContours(mask, contour, -1, (255,0,255), 1)

    @staticmethod
    def region_of_interest(img, vertices):
        mask = np.zeros_like(img)
        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    @staticmethod
    def draw_lines(img, lines, color=[255, 0, 0], thickness=10):
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    @staticmethod
    def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        LaneFollower.draw_lines(line_img, lines)
        return line_img

    @staticmethod
    def weighted_img(img, initial_img, α=0.8, β=0.6, γ=0.):
        return cv2.addWeighted(initial_img, α, img, β, γ)

    @staticmethod
    def stanley_control(cross_track_error, heading_error, speed, k, inverted=False):
        epsilon = 1e-6
        cross_track_term = math.atan2(k * cross_track_error, speed + epsilon)
        steering_angle = heading_error + cross_track_term
        return steering_angle

    def lane_finding_pipeline(self, img):
        if img is None:
            return None, None, None
        H, W = img.shape[:2]
        warp = 0.35
        mirror_point = 1 - warp
        src = np.float32([
            [W * warp, H],
            [0, 0],
            [W, 0],
            [W * mirror_point, H]
        ])
        # ... (source point visualization code remains the same)
        overlay_img_src = img.copy()
        src_int = src.astype(int)
        for i in range(4):
            cv2.circle(overlay_img_src, tuple(src_int[i]), 8, (0, 0, 255), -1)
        cv2.polylines(overlay_img_src, [src_int], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.imshow("Source Points Overlay", overlay_img_src)
        cv2.waitKey(1)
        
        dst_width, dst_height = 640, 480
        dst = np.float32([
            [0, dst_height], 
            [0, 0],
            [dst_width, 0], 
            [dst_width, dst_height]
        ])
        M = cv2.getPerspectiveTransform(dst, src)
        img_warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR)
        
        # --- Core Lane Detection Logic ---
        hsv_img = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)
        lower_hsv = (0, 120, 120)
        upper_hsv = (40, 255, 255)
        mask_yellow = cv2.inRange(hsv_img, lower_hsv, upper_hsv)
        masked_img = np.copy(img_warped)
        masked_img[mask_yellow == 0] = [0,0,0]
        
        gray_image = self.grayscale(masked_img)
        blurred_gray_img = self.gaussian_blur(gray_image, kernel_size=5)

        H, W = img_warped.shape[:2]
        region_height = H // 4
        
        vertices_regions = [
            np.array([[
                (0, H - (i + 1) * region_height),
                (0, H - i * region_height),
                (W, H - i * region_height),
                (W, H - (i + 1) * region_height)
            ]], dtype=np.int32) for i in range(3)
        ]
        masks = [self.region_of_interest(blurred_gray_img, vertices) for vertices in vertices_regions]
        
        contours_data = [self.get_contour_data(mask) for mask in masks]
        centers = [line for line, contours in contours_data if line]
        
        centers_sorted = sorted(centers, key=lambda c: -c['y'])
        bottom_two_centers = centers_sorted[:2]
        
        # --- Visualization and Control Calculation ---
        visualization_overlay = np.zeros_like(img_warped)

        # ✨ NEW: Draw all detected contours ✨
        for _, contours in contours_data:
            self.draw_contours(visualization_overlay, contours)

        # Draw all detected centers
        for center_data in centers:
            center_point = (center_data['x'], center_data['y'])
            cv2.circle(visualization_overlay, center_point, 5, (0, 255, 0), 7) 

        # Proceed only if we have two centers
        if len(bottom_two_centers) == 2:
            c1_data, c2_data = bottom_two_centers
            c1 = (c1_data['x'], c1_data['y'])
            c2 = (c2_data['x'], c2_data['y'])
            
            cv2.line(visualization_overlay, c1, c2, (255, 0, 0), 2)

            dx = c2[0] - c1[0]
            dy = c2[1] - c1[1]

            if dx != 0:
                yaw_deg = 90.0 - math.degrees(math.atan2(-dy, dx))
                yaw_stan = math.radians(yaw_deg)
                # yaw_deg = (yaw_deg + 180) % 360 - 90
                m = dy / dx
                b = c1[1] - m * c1[0]
                y1_vis = int(m * 0 + b)
                y2_vis = int(m * W + b)
                cv2.line(visualization_overlay, (0, y1_vis), (W, y2_vis), (0, 255, 0), 1)

                bottom_center = (W // 2, H)
                # Calculate the perpendicular slope
                perp_m = -1 / m if m != 0 else float('inf')
                # Calculate the y-intercept of the perpendicular line passing through bottom_center
                perp_b = bottom_center[1] - perp_m * bottom_center[0] if perp_m != float('inf') else 0
                # Find intersection point between the two lines
                if perp_m != float('inf'):
                    x_int = (perp_b - b) / (m - perp_m)
                    y_int = m * x_int + b
                else:
                    x_int = bottom_center[0]
                    y_int = m * x_int + b
                intersection = (int(x_int), int(y_int))
                cv2.line(visualization_overlay, bottom_center, intersection, (255, 0, 255), 1)
                # Calculate the distance in pixels from bottom_center to intersection
                dist = math.hypot(intersection[0] - bottom_center[0], intersection[1] - bottom_center[1])
                dist = dist * 0.725047081
                
                x_at_bottom = int((H-b)/m) if m != 0 else 0 # x = (y-b)/m with y = H
                cv2.circle(visualization_overlay, (x_at_bottom, H), 5, (255, 0, 255), -1)
                if x_at_bottom < W//2:
                    dist = -dist

                control = self.stanley_control(dist/100, yaw_stan, 1, 1)
                # Draw an arrow from the middle bottom of the frame showing the steering angle (90 - angle transformation)
                arrow_length = 100
                x0, y0 = W // 2, H
                angle_arrow = math.radians(90) - control  # 90 - angle transformation
                x1 = int(x0 + arrow_length * math.cos(angle_arrow))
                y1 = int(y0 - arrow_length * math.sin(angle_arrow))
                cv2.arrowedLine(visualization_overlay, (x0, y0), (x1, y1), (0, 0, 255), 3, tipLength=0.2)
                cv2.putText(visualization_overlay, f"yaw_err: {yaw_deg:.1f}deg {yaw_stan:.1f}rad", (10, H-50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
                cv2.putText(visualization_overlay, f"dist: {dist:.1f}cm", (10, H-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
                cv2.putText(visualization_overlay, f"ctrl: {control:.2f}rad", (10, H-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        final_result = self.weighted_img(visualization_overlay, img_warped, α=0.9, β=0.7, γ=0.0)

        if self.detect_aruco:
            detected, corners, ids = self.aruco_detector.detect(img)
            
        return (final_result, masks[0], img)

def main():
    # calibration = CameraCalibration('./picam_calibration_720p3.yaml')
    # aruco_detector = ArucoDetector(calibration)
    lane_follower = LaneFollower()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    profile = pipeline.start(config)
    
    output = 'acc_7.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    result_writer = cv2.VideoWriter(output, fourcc, 30.0, (640 * 2, 480))
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            cv2.imshow('RealSense', frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            result, mask, detected_aruco = lane_follower.lane_finding_pipeline(frame)
            if result is not None:
                cv2.imshow('detected_aruco', detected_aruco)
                cv2.imshow('frame', result)
                k = cv2.waitKey(1) & 0xFF
                frame_resized = cv2.resize(frame, (640, 480))
                result_resized = cv2.resize(result, (640, 480))
                stacked = np.hstack((frame_resized, result_resized))
                result_writer.write(stacked)
                if k == ord('s'):
                    cv2.imwrite('image.png', frame_resized)
                    cv2.imwrite('mask.png', mask)
                elif k == ord('q'):
                    break
                elif k == ord('p'):
                    while True:
                        cv2.imshow('detected_aruco', detected_aruco)
                        cv2.imshow('frame', result)
                        k = cv2.waitKey(1) & 0xFF
                        if k == ord('q'):
                            break
                        elif k == ord('p'):
                            break
                        elif k == ord('c'):
                            cv2.imwrite("captured_frame.png", frame_resized)
            else:
                blank = np.zeros((480, 640 * 2, 3), dtype=np.uint8)
                result_writer.write(blank)
    except Exception as e:
        print(f"Error starting RealSense pipeline: {e}")
        # return
    finally:
        cap.release()
        result_writer.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
