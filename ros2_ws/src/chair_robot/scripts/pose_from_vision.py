#!/usr/bin/env python3

import cv2 
import sys
import os
import yaml
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import TransformStamped
import queue
from pose_from_aruco import VideoProcess
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from ament_index_python import get_package_share_directory

model_path = '/absolute/path/to/pose_landmarker.task'

# Parameters for Realsense Color channel, 
RS_INTRINSIC_COLOR_640 = np.array([
    [615.21,0,310.90],[0,614.45,243.97],[0,0,1]
])

RS_DIST_COLOR_640 = np.array([0,0,0,0,0])

# CV options
BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

class PoseFromVision(Node):
    
    def __init__(self):
        self.node = node
        self.name = name
        self.use_gui = use_gui
        self.cap = cv2.VideoCapture(int(0))
        if not self.cap.isOpened():
            print("Could not open video")
            sys.exit()

        self.pose_pub = self.node.create_publisher(Float32MultiArray,"/marker_pose",10)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.detect_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict,detectorParams=self.detect_params)

        self.camMatrix = RS_INTRINSIC_COLOR_640
        self.distCoeffs = RS_DIST_COLOR_640

        # mediapipe landmarker
        self.cv_result = mp.tasks.vision.PoseLandmarkerResult
        self.landmarker = mp.tasks.vision.PoseLandmarker
        self.create_landmarker()
        self.cap = self.create_cap(attempt=0)

        # For testing
        self.timer = self.create_timer(.2, self.detect)

    def detect(self):
        # pull frame from cv2
        _, frame = self.cap.read()
        frame = cv2.flip(frame, 1)  # pylint: disable=no-member

        # draw the landmarks on the page for visualization
        landmarked_frame = draw_landmarks_on_image(frame, self.cv_result)
        cv2.imshow("frame", landmarked_frame)
        if frame is not None:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
            # detect landmarks
            self.landmarker.detect_async(
                image=mp_image, timestamp_ms=int(time.time() * 1000)
            )

    def create_cap(self, attempt):
        """
        creates the videocapture element and checks for failed video opening.

        Args:
            attempt: an integer representing the current attempt
        """
        cap = cv2.VideoCapture(0)  # pylint: disable=no-member
        time.sleep(0.5)
        if cap.isOpened():
            return cap
        elif attempt == 10:
            print("video capture FAILED, closing")
        else:
            print("video capture open failed, please restart")
            self.create_cap(attempt=attempt + 1)


    def create_landmarker(self):
        """
        Initializes the mediapipe landmarker object from the pose landmarker.task
        in livestream mode.

        Parameters resource
        https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker/python#configuration_options
        """
        share_dir = get_package_share_directory("chair_robot")
        config_path = os.path.join(share_dir,"config","pose_landmarker_lite.task")

        # callback function to grab latest cv result
        def update_result(
            result: mp.tasks.vision.HandLandmarkerResult,  # type: ignore
            output_image: mp.Image,  # pylint: disable=unused-argument
            timestamp_ms: int,  # pylint: disable=unused-argument
        ):
            self.cv_result = result

        options = mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(
                model_asset_path=config_path
            ),  # path to model
            running_mode=VisionRunningMode.LIVE_STREAM,  # running live stream
            min_pose_detection_confidence=0.5,
            min_pose_presence_confidence=0.5,
            min_tracking_confidence=0.5,
            result_callback=update_result,
        )

        # initialize landmarker from options
        self.landmarker = self.landmarker.create_from_options(options)


# From mediapipe example: https://colab.research.google.com/github/googlesamples/mediapipe/blob/main/examples/pose_landmarker/python/%5BMediaPipe_Python_Tasks%5D_Pose_Landmarker.ipynb#scrollTo=s3E6NFV-00Qt
def draw_landmarks_on_image(rgb_image, detection_result):
  pose_landmarks_list = detection_result.pose_landmarks
  annotated_image = np.copy(rgb_image)

  # Loop through the detected poses to visualize.
  for idx in range(len(pose_landmarks_list)):
    pose_landmarks = pose_landmarks_list[idx]

    # Draw the pose landmarks.
    pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
    pose_landmarks_proto.landmark.extend([
      landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
    ])
    solutions.drawing_utils.draw_landmarks(
      annotated_image,
      pose_landmarks_proto,
      solutions.pose.POSE_CONNECTIONS,
      solutions.drawing_styles.get_default_pose_landmarks_style())
  return annotated_image

if __name__ == '__main__':
    rclpy.init(args=args)
    pose_from_vision = PoseFromVision()
    rclpy.spin(pose_from_vision)
    pose_from_vision.destroy_node()
    rclpy.shutdown()
