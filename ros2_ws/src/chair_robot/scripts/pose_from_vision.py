#!/usr/bin/env python3

import sys
import time
import os
import cv2
import mediapipe as mp
import numpy as np

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import TransformStamped
from ament_index_python import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor


share_dir = get_package_share_directory("chair_robot")
model_path = os.path.join(share_dir,"config","pose_landmarker_lite.task")

# Parameters for Realsense Color channel, 
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()
fps_avg_frame_count = 10

FOCAL_LENGTH = 582
AV_WIDTH = 36 # cm

class PoseFromVision(Node):
    
    def __init__(self,channel):
        super().__init__('pose_from_vision')
        self.declare_parameter("channel",4)

        #topic to publish the pose to track 
        self.pose_topic = self.create_publisher(TransformStamped,"/pose_updates", 10)

        self.channel = self.get_parameter("channel").get_parameter_value().integer_value

        # mediapipe landmarker
        self.cv_result = mp.tasks.vision.PoseLandmarkerResult(pose_landmarks=[], pose_world_landmarks=[], segmentation_masks=[])
        detector, cap = self.setup()
        self.detector = detector
        self.cap = cap

        # For testing
        #self.timer = self.create_timer(0.01, self.detect)
        self.detect()

    def detect(self):
        self.detector.detect_async(mp_image, time.time_ns() // 1_000_000)
        while self.cap.isOpened():
            # flush buffer --- we want to make sure we grab a recent frame
            for _ in range(3):
                self.cap.grab()

            # pull frame from cv2
            ret, frame = self.cap.read()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                del(self)
            
            #frame = cv2.flip(frame, 1)  # pylint: disable=no-member
            if not ret:
                print("failed to get frame")
                return

            # draw the landmarks on the page for visualization
            landmarked_frame = self.draw_landmarks_on_image(frame, self.cv_result)

            success, image = self.cap.read()
            image = cv2.flip(image,1)

            # Convert the image from BGR to RGB as required by the TFLite model.
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

            print(f"FPS is: {FPS}")
            current_frame = image

            if self.cv_result.pose_landmarks != []:
                #left and right hip int vals
                a = 24
                b = 23
                x1 = self.cv_result.pose_landmarks[0][a].x
                y1 = self.cv_result.pose_landmarks[0][a].y
                x2 = self.cv_result.pose_landmarks[0][b].x
                y2 = self.cv_result.pose_landmarks[0][b].y
                p1 = np.array([x1, y1])
                p2 = np.array([x2, y2])
                hip_distance = np.linalg.norm([p1 - p2])

                #See: https://medium.com/@susanne.thierfelder/create-your-own-depth-measuring-tool-with-mediapipe-facemesh-in-javascript-ae90abae2362
                # depth = real-world-val * focal / measured-pxl-width
                z = FOCAL_LENGTH * AV_WIDTH / hip_distance
                #print(f"depth={z}\n")
                #take the average of the hip positions
                r23 = self.cv_result.pose_world_landmarks[0][a]
                r24 = self.cv_result.pose_world_landmarks[0][b]
                x = (r23.x + r24.x)/2
                y = (r23.y + r24.y)/2
                #send the message on topic
                msg = TransformStamped()
                msg.transform.translation.x = x
                msg.transform.translation.y = y
                msg.transform.translation.z = z
                self.pose_topic.publish(msg)
                #print(f"{self.cv_result.pose_world_landmarks[0][22]}")
            cv2.imshow('pose_landmarker', current_frame)
            cv2.waitKey(1)

    def setup(self):
        # setup video capture
        cap = self.create_cap(self.channel)
    
        # mediapipe
        def save_result(result: vision.PoseLandmarkerResult,
                            unused_output_image: mp.Image, timestamp_ms: int):
                global FPS, COUNTER, START_TIME
        
                # Calculate the FPS
                if COUNTER % fps_avg_frame_count == 0:
                    FPS = fps_avg_frame_count / (time.time() - START_TIME)
                    START_TIME = time.time()
        
                self.cv_result = result
                COUNTER += 1
    
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.LIVE_STREAM,
            num_poses=1,
            min_pose_detection_confidence=.4,
            min_pose_presence_confidence=.4,
            min_tracking_confidence=.4,
            output_segmentation_masks=False,
            result_callback=save_result)
        detector = vision.PoseLandmarker.create_from_options(options)
    
        return detector, cap

        print(f"cv_resuilt is: self.cv_result")
        # draw the landmarks on the page for visualization
        landmarked_frame = draw_landmarks_on_image(frame, self.cv_result)
        cv2.imshow("frame", landmarked_frame)


    def create_cap(self, attempt):
        """
        creates the videocapture element and checks for failed video opening.

        Args:
            attempt: an integer representing the current attempt
        """
        cap = cv2.VideoCapture(self.channel)  # pylint: disable=no-member
        time.sleep(0.5)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            cap.set(cv2.CAP_PROP_FPS, 15)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return cap
        elif attempt == 10:
            print("video capture FAILED, closing")
        else:
            print("video capture open failed, please restart")
            self.create_cap(attempt=attempt + 1)

    def draw_landmarks_on_image(self, rgb_image, detection_result):
        for pose_landmarks in self.cv_result.pose_landmarks:
            # Draw the pose landmarks.
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y,
                                                z=landmark.z) for landmark
                in pose_landmarks
            ])
            mp_drawing.draw_landmarks(
                rgb_image,
                pose_landmarks_proto,
                mp_pose.POSE_CONNECTIONS,
                mp_drawing_styles.get_default_pose_landmarks_style())
    
        return rgb_image

if __name__ == '__main__':
    if len(sys.argv) < 2:
        channel = 4
    else:
        channel = int(sys.argv[1])
    rclpy.init()
    pose_from_vision = PoseFromVision(channel=channel)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(pose_from_vision)
    executor.spin
    #rclpy.spin(pose_from_vision)
    #rclpy.shutdown()
