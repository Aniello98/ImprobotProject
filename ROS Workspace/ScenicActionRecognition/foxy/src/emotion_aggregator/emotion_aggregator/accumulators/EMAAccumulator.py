from typing import List
import numpy as np


def convert_msg_to_numpy(msg):
        """
        Converts the received message containing emotional scores into a numpy array
        :param msg: the object representing the message
        :return: the converted numpy array
        """
        scores_array = [
            msg.anger, msg.disgust, msg.fear, msg.happiness, msg.neutral, msg.sadness, msg.surprise
        ]
        return np.array(scores_array)

class EMAAccumulator:
    __alpha: float

    __window: List
    __window_size: int
    __smoothed_data: np.ndarray

    def __init__(self, window_size: int, data_len: int,  alpha: float):
        self.__alpha = alpha
        self.__data_len = data_len

        self.__window = []
        self.__window_size = window_size
        self.__smoothed_data = np.zeros(data_len)

    def add_prediction(self, data) -> np.ndarray:
        # Convert the ROS message into an array of confidence values
        data = convert_msg_to_numpy(data)
        # Then insert in the window
        self.__window.insert(0, data)
        self.__window = self.__window[:self.__window_size]
        

        for i in range(self.__data_len):
            factor = 1.0
            top_sum = 0.0
            bottom_sum = 0.0
            for data in self.__window:
                value = data[i]

                top_sum += factor * value
                bottom_sum += factor

                # Update factor.
                factor *= (1.0 - self.__alpha)

            self.__smoothed_data[i] = top_sum / bottom_sum

    def get_value(self):
        return self.__smoothed_data

    def reset(self):
        # Here we just flush data in the window
        self.__window.clear()
