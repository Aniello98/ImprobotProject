import cv2
from PIL import Image as PIL_Image
import threading
import numpy as np
import rclpy
import logging

from vision.VisionPublisher import VisionPublisher
from vision.eye_contact.EyeContactDetector import EyeContactDetector
from vision.face_emo.EmotionProcessor import EmotionProcessor
import mediapipe as mp
from torchvision import transforms
from paz.pipelines import MiniXceptionFER

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)

# Reference mediapipe namespaces
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose
mp_face_detection = mp.solutions.face_detection

TARGET_FRAME_H = 480
POSE_VISIBILITY_THRESHOLD = 50
USED_JOINTS = [0, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]
VIS_THRES = 0.6


def threaded(fn):
    def wrapper(*args, **kwargs):
        threading.Thread(target=fn, args=args, kwargs=kwargs).start()
    return wrapper


def get_visibility_ratio(pose) -> float:
    landmarks = pose.pose_landmarks.landmark
    # Retain only relevant joints
    reduced_joints = [landmarks[x] for x in USED_JOINTS]
    # Get the number of joints whose visibility is higher than the threshold
    visible_joints = sum(l.visibility > VIS_THRES for l in reduced_joints)
    # Return the visibility ratio as percentage
    return visible_joints / len(USED_JOINTS)*100


class VisionNode:

    __alive: bool
    __publisher: VisionPublisher

    def __init__(self):

        # Connect to webcam
        self.__cap = cv2.VideoCapture(0)
        self.__cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Create detectors for face and pose
        #self.__emo_detector = MiniXceptionFER()
        self.__emo_detector = EmotionProcessor()
        self.__face_detector = mp_face_detection.FaceDetection(
            model_selection=1, min_detection_confidence=0.5)
        self.__pose_estimator = mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.5)
        # Create eye contact detector
        self.__eye_contact_detector = EyeContactDetector()
        # Define the image transforms to process eye contact input
        self.test_transforms = transforms.Compose([transforms.Resize(224), transforms.CenterCrop(224), transforms.ToTensor(),
                                                   transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])
        self.__publisher = VisionPublisher()
        # Start threads
        self.__alive = True
        self.get_camera_frames()

    @threaded
    def get_camera_frames(self):
        while(self.__cap.isOpened() and self.__alive):
            # Get frame
            ret, frame = self.__cap.read()
            if not ret or frame is None:
                continue

            # Resize to target shape
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            greyFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            
            h, w, _ = frame.shape
            gh, gw = greyFrame.shape
            frame = cv2.resize(
                frame, (TARGET_FRAME_H, int(TARGET_FRAME_H*h/w)))
            greyFrame = cv2.resize(greyFrame, (self.__emo_detector.emotionTargetSize))
            
            frame.flags.writeable = False

            self.estimate_pose(frame)
            face = self.get_face_in_frame(frame)

            if face is None:
                self.analzye_facial_emotion(None)
                self.analyze_eye_contact(None)
            else:
                img = self.test_transforms(PIL_Image.fromarray(face))
                img.unsqueeze_(0)
                self.analyze_eye_contact(img)
                self.analzye_facial_emotion(face)

    def get_face_in_frame(self, frame):

        """
        Runs a face detection model to determine whether a face is present in the given frame.
        :return None if no face is present, the cropped face patch otherwise
        """
        # Detect a face
        face_results = self.__face_detector.process(frame)
        if face_results.detections:
            # Get the detected bounding box
            face_bb = face_results.detections[0].location_data.relative_bounding_box
            h, w, _ = frame.shape
            # Map the bounding box into the original frame for cropping
            # In case a coordinate is negative, put to 0
            face_x = max(int(face_bb.xmin * w), 0)
            face_y = max(int(face_bb.ymin * h), 0)
            face_w = int(face_bb.width * w)
            face_h = int(face_bb.height * h)
            # eventually crop the face patch
            face_pic = frame[face_y:face_y+face_h, face_x:face_x+face_w]

            # Send data to move base control manager
            # For simplicity change the refering system of the x to be sent to the move base control system
            #print(f"here is the range w: {w}")
            actor_orientation = self.transform_orientation(range = w, x = face_x)
            self.__publisher.publish_actor_orientation(actor_orientation)

            return face_pic
        else:
            actor_orientation = 3000 # Code to indicate that no faces are being detected
            self.__publisher.publish_actor_orientation(actor_orientation)
            return None

    def transform_orientation(self, range, x):
        ''' Transforms the x position with the reference system starting from the edge of the range, to the reference system with origin at the middel of range'''
        center = range / 2
        new_x = center - x 
        return new_x

    @threaded
    def analzye_facial_emotion(self, face):

        #logger = logging.getLogger("log")
        #logging.info("in analyze facial emotion")

        # Create a default neutral emotion to use as fallback result
        result = [0, 0, 0, 0, 0, 0, 1]
        if face is not None:
            face = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
            face = cv2.resize(face, (self.__emo_detector.emotionTargetSize))
            face = face.astype('float32')
            face = face / 255.0
            face = (face - 0.5) * 2.0
            face = np.expand_dims(face, 0)
            face = np.expand_dims(face, -1)

            # Get model result
            #model_result = self.__emo_detector(face)["scores"][0]
            model_result = self.__emo_detector(face) # model_result -> [AngryScore DisgustScore FearScore HappyScore SadScore SurpriseScore NeutralScore]
            #if model_result is not None and len(model_result) != 0:
            if model_result is not None :
                # And if valid replace the fallback neutral emotion
                result = model_result
        # Eventually publish the emotion
        self.__publisher.publish_emotion_scores(result)

    @threaded
    def analyze_eye_contact(self, face):
        result = self.__eye_contact_detector.detect_eye_contact_on_face(face)
        # Here we publish in any case, since when no face is detected it means no eye contact is present
        # Hence publish the float number on the topic
        self.__publisher.publish_eye_contact(result)

    # ===== POSE ESTIMATION =====
    @threaded
    def estimate_pose(self, img):
        # Perform inference on mediapipe
        results = self.__pose_estimator.process(cv2.flip(img, 0))
        if not results.pose_landmarks is None:
            # Here get visibilities of reduced set of joints
            visibility_ratio = get_visibility_ratio(results)
            if visibility_ratio >= POSE_VISIBILITY_THRESHOLD:
                # Publish the pose as is, converting into a NP matrix
                reduced_pose = [results.pose_landmarks.landmark[x]
                                for x in USED_JOINTS]
                reduced_pose = np.array([np.array([l.x, l.y])
                                        for l in reduced_pose])
                # Publish normalized pose
                self.__publisher.publish_pose(reduced_pose)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()


if __name__ == "__main__":
    main()
