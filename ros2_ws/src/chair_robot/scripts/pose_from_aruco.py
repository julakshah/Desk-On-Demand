#!/usr/bin/env python3

import cv2 
import sys
import numpy as np
import rclpy
from std_msgs.msg import Float32, Float32MultiArray
from rclpy.node import Node

# Parameters for Realsense Color channel, 
RS_INTRINSIC_COLOR_640 = np.array([
    [615.21,0,310.90],[0,614.45,243.97],[0,0,1]
])

RS_DIST_COLOR_640 = np.array([0,0,0,0,0])

class VideoProcess(Node):
    def __init__(self,use_gui,channel):
        super().__init__("video_processor")
        self.use_gui = use_gui
        self.cap = cv2.VideoCapture(int(channel))
        if not self.cap.isOpened():
            print("Could not open video")
            sys.exit()

        self.pose_pub = self.create_publisher(Float32MultiArray,"/marker_pose",10)
        self.timer = self.create_timer(0.01,self.timer_callback)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.detect_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict,detectorParams=self.detect_params)

        marker_length = 100 # 100 mm
        self.obj_pt_arr = np.asarray([
            [-marker_length/2.0,marker_length/2.0,0],
            [marker_length/2.0,marker_length/2.0,0],
            [marker_length/2.0,-marker_length/2.0,0],
            [-marker_length/2.0,-marker_length/2.0,0]
        ])

        self.camMatrix = RS_INTRINSIC_COLOR_640
        self.distCoeffs = RS_DIST_COLOR_640

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Could not read frame")
            return

        corners, ids, rejected = self.detector.detectMarkers(frame)

        img_clone = frame
        has_found_tag = False

        if len(corners) < 1:
            # we haven't found a marker, exit early
            # print("No marker found")
            return

        n_markers = len(corners[0])
        for i in range(n_markers):
            _, rvecs, tvecs = cv2.solvePnP(
                objectPoints=self.obj_pt_arr,
                imagePoints=corners[i],
                cameraMatrix=self.camMatrix,
                distCoeffs=self.distCoeffs,
            )

        msg = Float32MultiArray()
        unwrapped_tvecs = [tvecs[0,0],tvecs[1,0],tvecs[2,0]]
        #print(unwrapped_tvecs)
        msg.data = unwrapped_tvecs
        self.pose_pub.publish(msg)

        #print(f"self use gui: {self.use_gui}")
        if self.use_gui:
            img_clone = cv2.aruco.drawDetectedMarkers(img_clone,corners,ids)
            for i in range(len(ids)):
                cv2.drawFrameAxes(image=img_clone,
                    cameraMatrix=self.camMatrix,
                    distCoeffs=self.distCoeffs,
                    rvec=rvecs,
                    tvec=tvecs,
                    length=100.0,
                    thickness=10
                )
            cv2.imshow("Camera Feed",img_clone)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                del(self)


if __name__ == "__main__":
    print(f"len sys argv {len(sys.argv)}")
    if len(sys.argv) < 2:
        gui = 0
    else:
        gui = bool(int(sys.argv[1]))
        print(f"gui: {gui}")

    if len(sys.argv) < 3:
        channel = 8
    else:
        channel = sys.argv[2]

    rclpy.init()
    print("starting spin")
    rclpy.spin(VideoProcess(gui,channel))
    rclpy.shutdown()