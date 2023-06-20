from abc import ABC, abstractmethod

from impro_msgs.msg import EmotionScores
import numpy as np

EMOTIONS = 7


class EmotionAccumulator(ABC):
    _scores_embedding = np.zeros(EMOTIONS)

    def __init__(self):
        pass

    @staticmethod
    def __convert_msg_to_numpy(msg: EmotionScores):
        """
        Converts the received message containing emotional scores into a numpy array
        :param msg: the object representing the message
        :return: the converted numpy array
        """
        scores_array = [
            msg.anger, msg.disgust, msg.fear, msg.happiness, msg.neutral, msg.sadness, msg.surprise
        ]
        return np.array(scores_array)

    def add_prediction(self, emotion_scores: EmotionScores) -> None:
        # Convert the message into numpy format
        scores_array = self.__convert_msg_to_numpy(emotion_scores)
        # And update the embedding
        self._update_embedding(scores_array)

    def get_value(self):
        # Copy the current embedding
        out = np.copy(self._scores_embedding)
        # And return it
        return out

    @abstractmethod
    def _update_embedding(self, new_data):
        pass
