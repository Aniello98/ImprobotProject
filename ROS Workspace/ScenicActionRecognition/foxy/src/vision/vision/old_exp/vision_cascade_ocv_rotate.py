from typing import List
from typing import List
import cv2
from PIL import Image as PIL_Image

from torchvision import transforms
from scipy import ndimage

import numpy as np
from collections import deque
import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import mediapipe as mp
import ros2_numpy as rnp

import matplotlib.pyplot as plt

from impro_msgs.msg import EmotionScores
from vision.face_emo.EmotionProcessor import EmotionProcessor

from vision.eye_contact.EyeContactDetector import EyeContactDetector

from vision.pose.embedding.ReducedEmbedder import ReducedEmbedder

# Reference mediapipe namespaces
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

TARGET_FRAME_H = 480

DLIB_HOG_UPSAMPLING = 1

FACE_DETECTOR_PATH = "/home/airlab/lbpcascade_frontalface_improved.xml"

POSE_VISIBILITY_THRESHOLD = 50


def threaded(fn):
    def wrapper(*args, **kwargs):
        threading.Thread(target=fn, args=args, kwargs=kwargs).start()

    return wrapper


class VisionPublisher(Node):

    def __init__(self):
        super().__init__("vision_publisher")
        # Create publisher for emotion confidence values
        self.__scores_pub = self.create_publisher(
            EmotionScores, "/emotion_scores", 10)

        # Create the publisher for the pose. Since it will be encoded into a numpy matrix we publisha as an image
        self.__pose_pub = self.create_publisher(Image, "actor_pose", 10)

        self.__eye_contact_pub = self.create_publisher(
            Float64, "eye_contact", 10)

    def publish_emotion_scores(self, emotion_scores: List[float]) -> None:
        """
        Publish the detect emotions and corresponding confidence values
        :param emotion_scores: a dictionary containing the confidence value for each emotion
        """
        # Create the message object
        scores_msg = EmotionScores()
        scores_msg.anger = float(emotion_scores[0])
        scores_msg.disgust = float(emotion_scores[1])
        scores_msg.fear = float(emotion_scores[2])
        scores_msg.happiness = float(emotion_scores[3])
        scores_msg.neutral = float(emotion_scores[6])
        scores_msg.sadness = float(emotion_scores[4])
        scores_msg.surprise = float(emotion_scores[5])
        scores_msg.publisher.data = "face"
        # And publish on the topic
        self.__scores_pub.publish(scores_msg)

    def publish_pose(self, pose):
        msg = rnp.msgify(Image, pose, encoding="64FC1")
        self.__pose_pub.publish(msg)

    def publish_eye_contact(self, eye_contact_score):
        msg = Float64()
        msg.data = float(eye_contact_score)
        self.__eye_contact_pub.publish(msg)


class VisionNode:

    __alive: bool
    __publisher: VisionPublisher

    def __init__(self):

        # Conect to webcam
        self.__cap = cv2.VideoCapture(0)
        self.__cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Create face emotion process
        self.__emo_detector = EmotionProcessor()
        # Create pose embedder
        self.__pose_embedder = ReducedEmbedder()
        # Create face detector using OpenCV
        self.__face_detector = cv2.CascadeClassifier(FACE_DETECTOR_PATH)
        #self.__face_rot_angles = deque([0, 45, -45])
        self.__face_rot_angles = deque([0])
        # Create eye contact detector
        self.__eye_contact_detector = EyeContactDetector()

        self.__alive = True

        self.__publisher = VisionPublisher()

        self.test_transforms = transforms.Compose([transforms.Resize(224), transforms.CenterCrop(224), transforms.ToTensor(),
                                                   transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])

        # Start threads
        self.get_camera_frames()

    @threaded
    def get_camera_frames(self):
        ret = True
        while(ret and self.__alive):
            # Get frame
            ret, frame = self.__cap.read()
            if frame is None:
                continue
            
            # Resize to target shape
            h, w, _ = frame.shape
            frame = cv2.resize(
                frame, (TARGET_FRAME_H, int(TARGET_FRAME_H*h/w)))
            face, coords = self.get_face_in_frame(frame)
            self.estimate_pose(frame)
            
            #if face is not None:
            #    x, y, w, h = coords
            #    cv2.putText(frame, "FACCIA TROVATA", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA, False)
            #    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            #cv2.imshow("face", frame)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
            #    break

            if face is None:
                self.analzye_facial_emotion(None)
                self.analyze_eye_contact(None)
            else:
                img = self.test_transforms(PIL_Image.fromarray(face))
                img.unsqueeze_(0)
                self.analyze_eye_contact(img)
                self.analzye_facial_emotion(face)

            

            #time.sleep(0.01)

    def __get_face(self, frame, rotation_angle):
        rotated = ndimage.rotate(frame, rotation_angle)
        faces = self.__face_detector.detectMultiScale(rotated, 1.1, 4, minSize=(10, 10))
        return faces

    def get_face_in_frame(self, frame):
        """
        Runs a face detection model to determine whether a face is present in the given frame.
        :return None if no face is present, the bounding box of the face otherwise
        """
        found = False
        faces = None
        face_pic = None

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        for angle in self.__face_rot_angles:
            faces = self.__get_face(gray, angle)
            if len(faces) > 0:
                self.__face_rot_angles.remove(angle)
                self.__face_rot_angles.appendleft(angle)
                found = True
                break
        if found:
            (x, y, w, h) = faces[0]
            face_pic = ndimage.rotate(frame, angle)[y:y+h, x:x+w]
            return face_pic, (x, y, w, h)
        if not found:
            return None, None

    # ===== FACIAL EMOTIONS ===== #

    @threaded
    def analzye_facial_emotion(self, face):
        if face is None:
            # Create neutral emotion
            result = [0, 0, 0, 0, 0, 0, 1]
        # If no face is detected, we ignore
        else:
            result = self.__emo_detector(face)
            if result is None or len(result) == 0:
                # Create a neutral emotion in case of errors
                result = [0, 0, 0, 0, 0, 0, 1]
            self.__publisher.publish_emotion_scores(result)
        # TODO: Consider if this has to be done also for body emotion, but maybe not, since we are just anyway weighting the face more
    # ===== EYE CONTACT ===== #

    @threaded
    def analyze_eye_contact(self, face):
        result = self.__eye_contact_detector.detect_eye_contact_on_face(face)
        # Here we publish in any case, since when no face is detected it means no eye contact is present
        # Hence publish the float number on the topic
        self.__publisher.publish_eye_contact(result)

    # ===== POSE ESTIMATION =====

    def estimate_pose(self, img):
        # TODO: Check whether to put them as parameters
        with mp_pose.Pose(
                static_image_mode=False,
                model_complexity=1,
                min_detection_confidence=0.5) as pose:
            # Perform inference on mediapipe
            results = pose.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            if not results.pose_landmarks is None:
                # Here get visibilities of reduced set of joints
                visible_joints, visibility_ratio = self.__pose_embedder.get_visible_joints(results)
                if visibility_ratio >= POSE_VISIBILITY_THRESHOLD:
                    # Normalize the pose
                    _, pose_norm = self.__pose_embedder(results)
                    # Publish normalized pose
                    self.__publisher.publish_pose(pose_norm)
                # Otherwise do not publish the pose
                # TODO: Do not consider old poses when running body model


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()


if __name__ == "__main__":
    main()
