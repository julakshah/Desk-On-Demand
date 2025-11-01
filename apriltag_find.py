import cv2 
import sys
import numpy as np

RS_INTRINSIC_COLOR_1920 = np.array([
    [1384.23,0,939.51],[0,1384.52,548.93],[0,0,1]
])

RS_DIST_COLOR_1920 = np.array([0,0,0,0,0])

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open video")
        sys.exit()

    while 1:
        ret, frame = cap.read()
        if not ret:
            print("Could not read frame")
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        detect_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict,detectorParams=detect_params)
        corners = []
        ids = cv2.UMat()
        corners, ids, rejected = detector.detectMarkers(frame)
        print(f"corners: {corners}, ids: {ids}")

        marker_length = 100 # 100 mm
        obj_pt_arr = np.asarray([
            [-marker_length/2.0,marker_length/2.0,0],
            [marker_length/2.0,marker_length/2.0,0],
            [marker_length/2.0,-marker_length/2.0,0],
            [-marker_length/2.0,-marker_length/2.0,0]
        ])
        obj_points = cv2.Mat(obj_pt_arr)
        camMatrix = RS_INTRINSIC_COLOR_1920
        distCoeffs = RS_DIST_COLOR_1920

        img_clone = frame
        if len(corners) > 0: # if we found a marker
            n_markers = len(corners[0])
            print(n_markers)

            for i in range(n_markers):
                print("solving pnp")
                _, rvecs, tvecs = cv2.solvePnP(
                    objectPoints=obj_pt_arr,
                    imagePoints=corners[i],
                    cameraMatrix=camMatrix,
                    distCoeffs=distCoeffs,
                )

            img_clone = cv2.aruco.drawDetectedMarkers(img_clone,corners,ids)
            print(f"rvec: {rvecs}")
            print(f"tvec: {tvecs}")

            for i in range(len(ids)):
                cv2.drawFrameAxes(image=img_clone,
                    cameraMatrix=camMatrix,
                    distCoeffs=distCoeffs,
                    rvec=rvecs,
                    tvec=tvecs,
                    length=1.0,
                    thickness=1
                )

        cv2.imshow("Camera Feed",img_clone)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    main()