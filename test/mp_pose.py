import sys
import time

import cv2
import mediapipe as mp
import numpy as np

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2

mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()
DETECTION_RESULT = None
fps_avg_frame_count = 10

FOCAL_LENGTH = 582
AV_WIDTH = 36 # cm

    
def setup():
    # setup video capture
    cap = cv2.VideoCapture(0)

    # mediapipe
    def save_result(result: vision.PoseLandmarkerResult,
                        unused_output_image: mp.Image, timestamp_ms: int):
            global FPS, COUNTER, START_TIME, DETECTION_RESULT
    
            # Calculate the FPS
            if COUNTER % fps_avg_frame_count == 0:
                FPS = fps_avg_frame_count / (time.time() - START_TIME)
                START_TIME = time.time()
    
            DETECTION_RESULT = result
            COUNTER += 1

    base_options = python.BaseOptions(model_asset_path="./pose_landmarker_lite.task")
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


def main():
    detector, cap = setup()
    while True:
        success, image = cap.read()
        image = cv2.flip(image,1)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        detector.detect_async(mp_image, time.time_ns() // 1_000_000)
        print(f"FPS is: {FPS}")
        current_frame = image

        if DETECTION_RESULT and DETECTION_RESULT.pose_landmarks != []:
            ### debugging
            x1 = DETECTION_RESULT.pose_landmarks[0][23].x
            y1 = DETECTION_RESULT.pose_landmarks[0][23].y
            x2 = DETECTION_RESULT.pose_landmarks[0][24].x
            y2 = DETECTION_RESULT.pose_landmarks[0][24].y
            p1 = np.array([x1, y1])
            p2 = np.array([x2, y2])
            hip_distance = np.linalg.norm([p1 - p2])

            #See: https://medium.com/@susanne.thierfelder/create-your-own-depth-measuring-tool-with-mediapipe-facemesh-in-javascript-ae90abae2362
            # depth = real-world-val * focal / measured-pxl-width
            depth = FOCAL_LENGTH * AV_WIDTH / hip_distance


            print(f"depth={depth}\n")
            ###
            # Draw landmarks.
            for pose_landmarks in DETECTION_RESULT.pose_landmarks:
                # Draw the pose landmarks.
                pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
                pose_landmarks_proto.landmark.extend([
                    landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y,
                                                    z=landmark.z) for landmark
                    in pose_landmarks
                ])
                mp_drawing.draw_landmarks(
                    current_frame,
                    pose_landmarks_proto,
                    mp_pose.POSE_CONNECTIONS,
                    mp_drawing_styles.get_default_pose_landmarks_style())

        cv2.imshow('pose_landmarker', current_frame)
        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
