from typing import List

import rclpy
from rclpy.node import Node

import time

import numpy as np
import ros2_numpy as rnp

import tensorflow as tf

from sensor_msgs.msg import Image
from impro_msgs.msg import EmotionScores
from std_msgs.msg import Bool

from emo_body.embedding.PoseEmbedder import PoseEmbedder
from emo_body.embedding.MpEmbedder import MpEmbedder

from collections import deque

FRAME_LEN = 3  # Number of frames given in input to the network
MODEL_PATH = "/home/airlab/Documents/improbot/assets/body_emotion_nn_emb"

TIME_THRES = 5 # seconds

class BodyEmotionDetector(Node):

    __pose_accumulator: List[np.ndarray]
    __model: object  # The keras emotion model

    __pose_embedder: PoseEmbedder

    def __init__(self):
        super().__init__("body_emotion_detector")

        self.__pose_accumulator = []

        self.__enabled = True

        self.__last_write = None

        self.__pose_embedder = MpEmbedder()

        # Create the emotional model from pre-trained weights
        self.__model = tf.keras.models.load_model(MODEL_PATH)
        print(self.__model.summary())

        # Create the subscriber for the enable signal
        self.pose_sub = self.create_subscription(
            Image,
            "actor_pose",
            self.on_pose,
            10
        )

        self.create_subscription(
            Bool,
            "is_moving",
            self.on_feedback,
            10
        )

        # Create publisher for emotion scores
        self.__emotion_pub = self.create_publisher(
            EmotionScores, "emotion_scores", 10)

    def on_feedback(self, msg):
        self.__enabled = not msg.data
        if msg.data:
            # When the robot starts moving, flush collected emotions
            #self.get_logger().info("Flushing collected poses")
            self.__pose_accumulator.clear()


    def on_pose(self, pose_msg):
        
        if not self.__enabled:
            return
    
        # Time consistency
        if self.__last_write is None:
            self.__last_write = time.time()
        else:
            now = time.time()
            if now - self.__last_write > TIME_THRES:
                self.__pose_accumulator.clear()
                self.__last_write = now

        # Convert into numpy matrix
        pose_matrix = rnp.numpify(pose_msg)
        # Generate pose embedding
        embedding = self.__pose_embedder(pose_matrix)
        # Internally accumulate until we have enough frames
        self.__pose_accumulator.append(embedding)

        # Analysis is launched only here on a separate thread, to avoid race conditions
        if len(self.__pose_accumulator) == FRAME_LEN:
            # Create a matrix from the accumulated poses
            x = np.array(self.__pose_accumulator)
            # NOTE: we have to reshape, since network input is encoded like this 
            x = x.reshape((3, 25*2))
            # Expand into a 1-element batch
            x = np.expand_dims(x, 0)
            # Reset the accumulator
            self.__pose_accumulator = []
            # Start emotion inference in a separate thread
            self.__get_emotion_from_pose(x)

    def __publsh_emotion_from_body(self, confidence_values):
        # Build the emotion message
        msg = EmotionScores()
        msg.publisher.data = "body"
        msg.anger = float(confidence_values[0])
        msg.disgust = float(confidence_values[1])
        msg.fear = float(confidence_values[2])
        msg.happiness = float(confidence_values[3])
        msg.sadness = float(confidence_values[4])
        msg.surprise = float(confidence_values[5])
        msg.neutral = float(confidence_values[6])
        # Eventually publish in the topic
        self.__emotion_pub.publish(msg)

    def __get_emotion_from_pose(self, pose_sequence):
        # Run inference
        y = self.__model(pose_sequence)
        # Get array of confidence values
        confidence_values = y[0].numpy()
        # And add them to the buffer
        self.__publsh_emotion_from_body(confidence_values)


def main(args=None):
    rclpy.init(args=args)

    emotion_detector = BodyEmotionDetector()
    rclpy.spin(emotion_detector)


if __name__ == "__main__":
    main()
