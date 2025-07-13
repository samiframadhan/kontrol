import numpy
import cv2
from cv2 import aruco
import argparse

class CharucoCalibrator:
    def __init__(self, video_path, num_captures):
        self.video_path = video_path
        self.num_captures = num_captures

        self.CHARUCOBOARD_ROWCOUNT = 8
        self.CHARUCOBOARD_COLCOUNT = 6
        self.ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)
        self.CHARUCO_BOARD = aruco.CharucoBoard(
            (self.CHARUCOBOARD_COLCOUNT, self.CHARUCOBOARD_ROWCOUNT),
            squareLength=0.032,
            markerLength=0.015,
            dictionary=self.ARUCO_DICT)

        self.corners_all = []
        self.ids_all = []
        self.image_size = None

    def run(self):
        cap = cv2.VideoCapture(self.video_path)
        validCaptures = 0

        while cap.isOpened():
            ret, img = cap.read()
            if not ret:
                break

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(
                image=gray,
                dictionary=self.ARUCO_DICT)

            if ids is None:
                continue

            img = aruco.drawDetectedMarkers(image=img, corners=corners)
            response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=self.CHARUCO_BOARD)

            if response > 15:
                self.corners_all.append(charuco_corners)
                self.ids_all.append(charuco_ids)
                img = aruco.drawDetectedCornersCharuco(
                    image=img,
                    charucoCorners=charuco_corners,
                    charucoIds=charuco_ids)

                if not self.image_size:
                    self.image_size = gray.shape[::-1]

                proportion = max(img.shape) / 1000.0
                img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))
                cv2.imshow('Charuco board', img)
                if cv2.waitKey(0) == ord('q'):
                    break

                validCaptures += 1
                if validCaptures == self.num_captures:
                    break

        cap.release()
        cv2.destroyAllWindows()
        print("{} valid captures".format(validCaptures))

        if validCaptures < self.num_captures or len(self.corners_all) == 0:
            print("Calibration was unsuccessful. Not enough valid charucoboards detected.")
            exit()

        print("Generating calibration...")
        calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=self.corners_all,
            charucoIds=self.ids_all,
            board=self.CHARUCO_BOARD,
            imageSize=self.image_size,
            cameraMatrix=None,
            distCoeffs=None)

        print("Camera intrinsic parameters matrix:\n{}".format(cameraMatrix))
        print("\nCamera distortion coefficients:\n{}".format(distCoeffs))

        fs = cv2.FileStorage('./CameraCalibration.yml', cv2.FILE_STORAGE_WRITE)
        fs.write("camera_matrix", cameraMatrix)
        fs.write("dist_coeff", distCoeffs)
        fs.release()
        print('Calibration successful. Calibration file created: CameraCalibration.yml')

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", required=True, help="Path to the video file for calibration")
    ap.add_argument("-c", "--captures", required=True, help="Number of captures", type=int)
    args = vars(ap.parse_args())

    calibrator = CharucoCalibrator(args["video"], args["captures"])
    calibrator.run()
