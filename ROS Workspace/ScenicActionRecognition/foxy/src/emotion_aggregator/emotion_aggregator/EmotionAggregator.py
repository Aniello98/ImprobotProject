import time
from typing import Dict
from common.constants import *


import rclpy
from rclpy.node import Node

import numpy as np

from common.functions import threaded
from emotion_aggregator.fusion.WeightFusion import WeightFusion
from emotion_aggregator.fusion.EmotionFusionMethod import EmotionFusionMethod

from emotion_aggregator.accumulators.EMAAccumulator import EMAAccumulator
from impro_msgs.msg import EmotionScores, ProxemicZone
from std_msgs.msg import Bool

EMA_ALPHA = 0.3
EMOTIONS = 7


class EmotionAggregator(Node):
    __accumulators: Dict[str, EMAAccumulator]

    __alive: bool

    __fusion_method: EmotionFusionMethod

    def __init__(self):
        super(EmotionAggregator, self).__init__("emotion_aggregator")

        self.__accumulators = {}
        self.__alive = True
        self.__enabled = True

        # Initialize fusion method
        self.__fusion_method = WeightFusion()

        # Initialize subscribed to emotion confidence values
        self.emotion_sub = self.create_subscription(
            EmotionScores,
            "emotion_scores",
            self.on_emotion_scores,
            10
        )

        self.create_subscription(
            Bool,
            "is_moving",
            self.on_feedback,
            10
        )

        self.create_subscription(
            ProxemicZone,
            PROX_ZONE_TOPIC,
            self.on_zone,
            10
        )

        self.zone = None

        # Create publisher for final emotion
        self.emotion_pub = self.create_publisher(EmotionScores, "emotion", 10)

        #for debug
        self.face_emotion_pub = self.create_publisher(EmotionScores, "face_emotion", 10)
        self.body_emotion_pub = self.create_publisher(EmotionScores, "body_emotion", 10)


        # Start a thread that periodically fetches data and publishes emotion
        self.aggregator()

    def on_zone(self, data):
        self.zone = data.zone

    def on_feedback(self, msg):
        self.__enabled = not msg.data
        if msg.data:
            self.get_logger().info("Flushing emotion aggregator window upon receiving motion feedback")
            # When the robot starts moving, flush collected emotions
            #self.get_logger().info("Resetting accumulators before moving")
            # Call the reset method on all the accumulators
            [self.__accumulators[k].reset() for k in self.__accumulators]

    def on_emotion_scores(self, data):
        # Retrieve the publisher
        publisher_id: str = data.publisher.data

        # --- debug ---
        if publisher_id == "face":
            self.face_emotion_pub.publish(data)
        else:
            self.body_emotion_pub.publish(data)
        # --- debug --

        # Check if we have to initialize
        if publisher_id not in self.__accumulators:
            # Initialize an accumulator
            accumulator = EMAAccumulator(10, EMOTIONS, EMA_ALPHA)
            # And put in the table
            self.__accumulators[publisher_id] = accumulator
        # And eventually add the new data
        self.__accumulators[publisher_id].add_prediction(data)

    @threaded
    def aggregator(self):
        while self.__alive:
            # If the node is not enabled, or if no publisher has written, wait
            if not self.__enabled:
                time.sleep(0.5)
                continue
            if len(self.__accumulators) == 0:
                # If no data are available, emit neutral class
                fused_emotion = np.zeros(EMOTIONS)
                fused_emotion[-1] = 1
            elif len(self.__accumulators) == 1:
                # If only one is present, emit it
                fused_emotion = self.__accumulators[list(
                    self.__accumulators)[0]].get_value()
            # Otherwise fuse the face and body emotions
            else:
                # Get smoothened vectors from face and body
                face_emotion = self.__accumulators["face"].get_value()
                body_emotion = self.__accumulators["body"].get_value()


                if(not self.zone == 2): # if the actor is far more than 2.5 meters from the robot
                    fused_emotion = self.__fusion_method.fuse(
                        face_emotion, body_emotion)
                else:
                    fused_emotion = self.__fusion_method.fuse(
                        face_emotion, body_emotion, face_weight = 0)


            # Publish the aggregated scores
            self.publish_emotion_scores(fused_emotion)

            time.sleep(1)  # TODO: Understand right delay

    def publish_emotion_scores(self, emotion_scores) -> None:
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
        scores_msg.publisher.data = "aggregator"
        # And publish on the topic
        self.emotion_pub.publish(scores_msg)


def main(args=None):
    rclpy.init(args=args)

    aggregator = EmotionAggregator()
    rclpy.spin(aggregator)


if __name__ == "__main":
    main()
