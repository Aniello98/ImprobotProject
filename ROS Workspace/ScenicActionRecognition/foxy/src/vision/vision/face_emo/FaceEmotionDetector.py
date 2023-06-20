from typing import Deque, Dict, List
from collections import deque

import rclpy
import time
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from impro_msgs.msg import EmotionScores

from common.functions import threaded
import common.constants as const

from EmotionProcessor import EmotionProcessor


class FaceEmotionDetector(Node):
    # a buffer to store incoming images, to not block the comm channel while
    # performing emotion analysis
    __buffer: Deque

    __alive: bool

    def __init__(self):
        super().__init__("image_listener")

        self.__bridge = CvBridge()
        self.__emo_detector = EmotionProcessor()

        self.__buffer = deque()
        self.__alive = True

        # Create the subscriber for the webcam image
        self.create_subscription(
            Image,
            "image_raw",
            self.on_image,
            10
        )
        # Create publisher for emotion confidence values
        self.__scores_pub = self.create_publisher(EmotionScores, const.EMOTION_SCORES_TOPIC, 10)

    def on_image(self, image_msg):
        # First convert the message into an image format
        img = self.__bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        # And then add to buffer
        self.__buffer.append(img)

    def publish_emotion_scores(self, emotion_scores: List[float]) -> None:
        """
        Publish the detect emotions and corresponding confidence values
        :param emotion_scores: a dictionary containing the confidence value for each emotion
        """
        # Create the message object
        scores_msg = EmotionScores()
        scores_msg.anger = emotion_scores[0]
        scores_msg.disgust = emotion_scores[1]
        scores_msg.fear = emotion_scores[2]
        scores_msg.happiness = emotion_scores[3]
        scores_msg.neutral = emotion_scores[6]
        scores_msg.sadness = emotion_scores[4]
        scores_msg.surprise = emotion_scores[5]
        scores_msg.publisher.data = const.FACE_EMOTION_ID
        # And publish on the topic
        self.__scores_pub.publish(scores_msg)

    @threaded
    def analyze_facial_emotions(self):
        while self.__alive:
            if len(self.__buffer) == 0:
                time.sleep(0.01)
            else:
                img = self.__buffer.popleft()
                # then ask PAZ to detect emotions in the image
                result = self.__emo_detector(img)
                if len(result) != 0:
                    emotions: List[float] = result
                    self.publish_emotion_scores(emotions)


def main(args=None):
    rclpy.init(args=args)

    emotion_detector = FaceEmotionDetector()
    rclpy.spin(emotion_detector)


if __name__ == "__main__":
    main()
