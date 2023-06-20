from typing import List
import rclpy
from rclpy.node import Node

from impro_msgs.msg import EmotionScores

from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Int16
import ros2_numpy as rnp


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
        
        self.__actor_orientation_pub = self.create_publisher(
            Int16, "/actor_orientation", 10
        )

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
    
    def publish_actor_orientation(self, x_orientation):
        msg = Int16()
        msg.data = int(x_orientation)
        self.__actor_orientation_pub.publish(msg)