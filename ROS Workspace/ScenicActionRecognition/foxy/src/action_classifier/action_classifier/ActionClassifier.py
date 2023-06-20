import rclpy
from rclpy.node import Node

import numpy as np

import time


from std_msgs.msg import Bool

from action_classifier.FuzzySystem import FuzzySystem, NEUTRAL_INDEX, ACTIONS
from action_classifier.EmotionSmoother import EmotionSmoother

from action_classifier.WindowVoter import WindowVoter

import common.constants as const
from common.functions import threaded

from impro_msgs.msg import (
    EmotionScores,
    ProxemicMovement,
    ProxemicZone,
    ScenicAction
)

CLASSIF_STEP_DELAY = 0.5  # in seconds

OLD_DATA_THRESHOLD = 3  # in seconds


class ActionClassifier(Node):

    __fuzzy_system: FuzzySystem
    __emotion_smoother: EmotionSmoother

    __enabled: Bool

    __smoothing: Bool

    def __init__(self, smoothing: bool=False):
        super().__init__("action_classifier")

        # Initialize the fuzzy system
        self.__fuzzy_system = FuzzySystem()
        self.__fuzzy_system.add_printer(self)
        self.__emotion_smoother = EmotionSmoother()

        self.__zone = None
        self.__emotion = None
        self.__movement = None

        self.__alive = True
        self.__enabled = True

        self.__smoothing = smoothing

        self.__accumulator = WindowVoter(5, len(ACTIONS))

        # ===== SUBSCRIBERS ===== #

        # Create subscriber for emotion
        self.create_subscription(
            EmotionScores, const.EMOTION_TOPIC, self.on_emotion, 10)

        # Create subscriber for proxemic zone
        self.create_subscription(
            ProxemicZone, const.PROX_ZONE_TOPIC, self.on_zone, 10)

        # Create subscriber for proxemic movement
        self.create_subscription(
            ProxemicMovement, const.PROX_MVMT_TOPIC, self.on_movement, 10
        )

        # Create subscriber to trigger action classification
        self.create_subscription(
            Bool,
            "is_moving",
            self.on_feedback,
            10,
        )

        # Create publisher of classification result
        self.__action_pub = self.create_publisher(
            ScenicAction, "scenic_action", 10)

        # Start the classifier thread
        self.classification_loop()

    # TODO: Understand if smoothing has to be done also here, but maybe not since we are comparing the results of already smooth frames

    def print_msg(self, msg):
        self.get_logger().info(msg)

    def on_emotion(self, data):
        self.__emotion = [data, time.time()]

    def on_zone(self, data):
        self.__zone = [data, time.time()]

    def on_movement(self, data):
        self.__movement = [data, time.time()]

    def __convert_emotion_msg_to_np(self, emotion_msg):
        """
        Converts a message representing emotion confidence scores into a numpy vector
        """
        return np.array([
            emotion_msg.anger,
            emotion_msg.disgust,
            emotion_msg.fear,
            emotion_msg.happiness,
            emotion_msg.sadness,
            emotion_msg.surprise,
            emotion_msg.neutral
        ])

    def __data_are_recent(self) -> bool:
        """Determines if there are data that are too old to perform classification

        Returns:
            bool: True if all data are enough recent, False if there is at leason one that is too old 
        """
        # Get current time
        now = time.time()
        # Compute time difference between now and ingestion time
        time_differences = [self.__zone[1],
                            self.__emotion[1], self.__movement[1]]
        time_differences = [
            0 if t is None else now - t
            for t in time_differences
        ]
        # Check if all data are enough recent
        return all([t <= OLD_DATA_THRESHOLD for t in time_differences])

    def __data_are_available(self) -> bool:
        return (not self.__zone is None) and (not self.__emotion is None) and (not self.__movement is None)

    def __roundTraditional(self, val, digits=1):
        return round(val+10**(-len(str(val))-1), digits)

    @threaded
    def classification_loop(self):
        while self.__alive:

            # self.get_logger().info(
            #    f"Zone: {self.__zone}\t emotion: {self.__emotion}\tMov: {self.__movement}"
            # )

            # First check if all data are recent and the node is enabled
            if self.__enabled and self.__data_are_available() and self.__data_are_recent():
                #self.get_logger().info("Starting new classification loop")
                # First convert the emotion message into a numpy vector to perform smoothing
                np_emotion = self.__convert_emotion_msg_to_np(
                    self.__emotion[0])
                # Ask the emotion smoother to smooth the current emotion
                input_emotion = self.__emotion_smoother.smooth(np_emotion)
                try:
                    # Pass values to the fuzzy system
                    action = self.__fuzzy_system.classify(
                        zone=self.__zone[0].zone,
                        emotion_vector=input_emotion,
                        lin_mov=self.__movement[0].linear_speed)    
                    # Round the classified action to the nearest integer
                    rounded_action = int(self.__roundTraditional(action))
                except:
                    rounded_action = NEUTRAL_INDEX

                # If enabled, smooth the output
                if self.__smoothing:
                    self.__accumulator.add_prediction(rounded_action)
                    rounded_action = self.__accumulator.get_value()
                    rounded_action = int(self.__roundTraditional(rounded_action))
                # Build output message
                action_msg = ScenicAction()
                action_msg.action = rounded_action # sends the index of the output fuzzy ACTION
                action_label = self.__fuzzy_system.get_label(rounded_action)
                self.get_logger().info(action_label)
                action_msg.label.data = action_label
                # And publish
                self.__action_pub.publish(action_msg)
            else:
                # Otherwise publish a neutral action
                # Build output message
                action_msg = ScenicAction()
                action_msg.action = NEUTRAL_INDEX
                action_msg.label.data = "neutral"
                # And publish
                self.__action_pub.publish(action_msg)

            # Otherwise, don't do anything
            time.sleep(CLASSIF_STEP_DELAY)

    def on_feedback(self, msg):
        self.__enabled = not msg.data


def main(args=None):
    rclpy.init(args=args)

    action_classifier = ActionClassifier(smoothing=False)
    rclpy.spin(action_classifier)


if __name__ == "__main__":
    main()
