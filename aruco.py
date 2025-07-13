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