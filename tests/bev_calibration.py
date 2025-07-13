import cv2
import numpy as np
import cv2.aruco as aruco

def infer_and_get_source_points(corners, ids):
    """
    Infers the layout of 4 ArUco markers and returns their corners in a consistent order.

    Args:
        corners (list): List of corner coordinates for each detected marker.
        ids (np.ndarray): List of IDs for each detected marker.

    Returns:
        np.ndarray: An array of 4 source points (the corners of the overall rectangle)
                    ordered as [top-left, top-right, bottom-right, bottom-left].
                    Returns None if inference fails.
    """
    if len(corners) != 4:
        print("Inference failed: Please ensure exactly 4 markers are visible.")
        return None

    # Flatten the IDs and calculate the center point (centroid) for each marker
    ids = ids.flatten()
    centroids = np.empty((4, 2), dtype=np.int32)
    for i, corner_set in enumerate(corners):
        centroids[i] = np.mean(corner_set[0], axis=0).astype(int)

    # --- Infer corner positions based on geometry ---
    # Top-left has the smallest sum of coordinates (x+y)
    # Bottom-right has the largest sum of coordinates (x+y)
    s = np.sum(centroids, axis=1)
    tl_index = np.argmin(s)
    br_index = np.argmax(s)

    # Top-right has the smallest difference (y-x)
    # Bottom-left has the largest difference (y-x)
    diff = np.diff(centroids, axis=1) # y-x if axis is 1 and order is x,y
    tr_index = np.argmin(diff)
    bl_index = np.argmax(diff)

    # --- Get the correct corner from each identified marker ---
    # Map the inferred index back to the original marker corners list
    # The order is crucial for cv2.getPerspectiveTransform
    src_pts = np.array([
        corners[tl_index][0][0], # Top-left corner of the top-left marker
        corners[tr_index][0][1], # Top-right corner of the top-right marker
        corners[br_index][0][2], # Bottom-right corner of the bottom-right marker
        corners[bl_index][0][3], # Bottom-left corner of the bottom-left marker
    ], dtype=np.float32)

    print(f"✅ Inference successful! Marker IDs inferred as:")
    print(f"  - Top-Left:     ID {ids[tl_index]}")
    print(f"  - Top-Right:    ID {ids[tr_index]}")
    print(f"  - Bottom-Right: ID {ids[br_index]}")
    print(f"  - Bottom-Left:  ID {ids[bl_index]}")

    return src_pts

if __name__ == '__main__':
    # --- Configuration ---
    ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
    ARUCO_PARAMS = aruco.DetectorParameters()
    DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
    
    # Video capture
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open video stream.")
        exit()

    # Transformation matrix, initially None
    perspective_M = None
    
    # --- Main Loop ---
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Placeholders
        bev_frame = np.zeros((500, 500, 3), dtype=np.uint8)
        display_frame = frame.copy()

        # Detect markers for drawing and potential calibration
        corners, ids, _ = DETECTOR.detectMarkers(frame)
        if ids is not None:
            aruco.drawDetectedMarkers(display_frame, corners, ids)
        
        # Check for user input
        key = cv2.waitKey(1) & 0xFF

        # Calibrate on 'c' key press
        if key == ord('c'):
            print("\nAttempting calibration by inferring layout...")
            if ids is not None and len(ids) == 4:
                # 1. Infer the layout to get ordered source points
                src_points = infer_and_get_source_points(corners, ids)
                
                if src_points is not None:
                    # 2. Define the destination rectangle
                    dst_width, dst_height = 500, 500
                    dst_points = np.array([
                        [0, 0],
                        [dst_width - 1, 0],
                        [dst_width - 1, dst_height - 1],
                        [0, dst_height - 1]
                    ], dtype=np.float32)

                    # 3. Calculate the transformation matrix
                    perspective_M = cv2.getPerspectiveTransform(src_points, dst_points)
                    print("✅ Calibration matrix calculated.")
            else:
                print("Calibration failed: Ensure exactly 4 markers are clearly visible.")


        # Quit on 'q' key press
        elif key == ord('q'):
            break

        # If a transformation matrix exists, apply it
        if perspective_M is not None:
            bev_frame = cv2.warpPerspective(frame, perspective_M, (500, 500))

        # --- Display Results ---
        cv2.imshow("Original Feed (Press 'c' to calibrate)", display_frame)
        cv2.imshow("Bird's-Eye View (Press 'q' to quit)", bev_frame)

    # --- Cleanup ---
    cap.release()
    cv2.destroyAllWindows()