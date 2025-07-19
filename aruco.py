import cv2
import numpy as np
import zmq

class ArucoDetector:
    def __init__(self, calib_file, aruco_dict_type=cv2.aruco.DICT_5X5_50, marker_length=0.1, camera_id=0):
        self.camera_matrix, self.dist_coeffs = self.load_calibration(calib_file)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.parameters = cv2.aruco.DetectorParameters()
        self.marker_length = marker_length
        self.cap = cv2.VideoCapture(camera_id)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")
        self.robot_to_camera_transform = self.load_robot_to_camera_transform("robot_camera_transform.yaml")

    @staticmethod
    def load_robot_to_camera_transform(self, filename):
        fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
        mat = fs.getNode("robot_to_camera").mat()
        fs.release()
        if mat.shape == (4, 4):
            return mat
        else:
            raise ValueError("Expected 4x4 matrix in robot_to_camera_transform.yaml under 'robot_to_camera'")
        
    def compute_marker_position_in_robot_frame(self, tvec, rvec):
        """
        Transforms marker pose from camera frame to robot frame using a known static transform.
        Assumes robot_to_camera_transform is a 4x4 matrix in self.robot_to_camera_transform.
        """
        # Marker pose in camera frame (homogeneous)
        marker_in_camera = np.eye(4)
        marker_in_camera[:3, :3], _ = cv2.Rodrigues(rvec)
        marker_in_camera[:3, 3] = tvec.flatten()

        # Transform to robot frame
        marker_in_robot = self.robot_to_camera_transform @ marker_in_camera

        # Extract translation (x, y, z)
        position = marker_in_robot[:3, 3]
        distance = np.linalg.norm(position)
        return position, distance

    
    @staticmethod
    def load_calibration(filename):
        fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("distortion_coefficients").mat()
        fs.release()
        return camera_matrix, dist_coeffs
    
    def publish_marker(self, marker_id, position):
        self.socket.send_string("marker_data", flags=zmq.SNDMORE)
        self.socket.send_pyobj({"id": marker_id, "position": position})
        print(f"Marker ID: {marker_id}, Position: {position}")

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            corners, ids, rejected = cv2.aruco.detectMarkers(
                frame, self.aruco_dict, parameters=self.parameters
            )

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                )
                for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                    # Draw marker
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)

                    # Transform marker position to robot frame
                    position, distance = self.compute_marker_position_in_robot_frame(tvec, rvec)
                    x, y, z = position

                    corner = tuple(np.int32(corners[i][0][0]))
                    cv2.putText(
                        frame,
                        f"ID: {ids[i][0]}, Dist: {distance:.2f} m",
                        (corner[0], corner[1]),
                        cv2.FONT_HERSHEY_COMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )
                    self.publish_marker(ids[i][0], position)

                for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)
                    x, y, z = tvec[0]
                    corner = tuple(np.int32(corners[i][0][0]))
                    cv2.putText(
                        frame,
                        f"ID: {ids[i][0]}, Pos: ({x:.2f}, {y:.2f}, {z:.2f})",
                        (corner[0], corner[1]),
                        cv2.FONT_HERSHEY_COMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )
                    self.publish_marker(ids[i][0], (x, y, z))

            cv2.imshow("Aruco Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = ArucoDetector("calibrationc270.yaml", marker_length=0.049)
    detector.run()