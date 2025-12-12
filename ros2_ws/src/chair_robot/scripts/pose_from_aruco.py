#!/usr/bin/env python3

import cv2 
import sys
import os
import yaml
import numpy as np
import rclpy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from ament_index_python import get_package_share_directory

# Parameters for Realsense Color channel, 
RS_INTRINSIC_COLOR_640 = np.array([
    [615.21,0,310.90],[0,614.45,243.97],[0,0,1]
])

RS_DIST_COLOR_640 = np.array([0,0,0,0,0])

class VideoProcess:
    def __init__(self,node: Node,use_gui,channel,follow_target=-1,name="robot0"):
        self.node = node
        self.name = name
        self.use_gui = use_gui
        self.cap = cv2.VideoCapture(int(channel))
        if not self.cap.isOpened():
            print("Could not open video")
            sys.exit()

        self.pose_pub = self.node.create_publisher(Float32MultiArray,"/marker_pose",10)
        
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

        share_dir = get_package_share_directory("chair_robot")
        config_path = os.path.join(share_dir,"config","marker_ids.yaml")
        with open(config_path, 'r')as f:
            config = yaml.safe_load(f)
        print(f"Config: {config}")

        self.known_ids = config["ids"] # map tag ID to robot ID
        self.robot_names = config["names"] # map robot ID to robot name
        self.follow_target = follow_target # robot ID

        # List of valid tag IDs (all tag IDs which map to the follow target id)
        self.valid_ids = [k for k,v in self.known_ids.items() if v < self.follow_target]

        self.camMatrix = RS_INTRINSIC_COLOR_640
        self.distCoeffs = RS_DIST_COLOR_640

    def process_frame(self):
        pose_diffs = {}
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
            if self.use_gui:
                cv2.imshow("Camera Feed",img_clone)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    del(self)
            return
        else:
            print(f"Got corners!")

        n_markers = len(corners[0])
        for i in range(n_markers):
            _, rvecs, tvecs = cv2.solvePnP(
                objectPoints=self.obj_pt_arr,
                imagePoints=corners[i],
                cameraMatrix=self.camMatrix,
                distCoeffs=self.distCoeffs,
            )
            robot_id = ids[i].squeeze() # ID of robot, given tag
            print(f"Known ids: {ids}")
            if robot_id in self.known_ids: 
                name = self.known_ids[robot_id]
                pose_diffs[name] = [rvecs,tvecs]

        msg = Float32MultiArray()
        unwrapped_tvecs = [tvecs[0,0],tvecs[1,0],tvecs[2,0]]
        print(unwrapped_tvecs)
        msg.data = unwrapped_tvecs

        # Publish pose difference
        t = TransformStamped()
        t.header.frame_id = self.name # our name
        t.child_frame_id = self.robot_names[self.follow_target] # our target's name

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
        
        return pose_diffs


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
    node = Node("video_process")
    vid_process = VideoProcess(node,gui,channel)
    print("starting spin")
    #rclpy.spin(node)

    while True:
        try:
            #rclpy.spin_once(node)
            pose_diffs = vid_process.process_frame()
            print(f"Pose diffs: {pose_diffs}")
        except KeyboardInterrupt:
            rclpy.shutdown()
            sys.exit()