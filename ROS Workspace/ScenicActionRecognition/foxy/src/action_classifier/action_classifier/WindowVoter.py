from typing import List
import numpy as np


class WindowVoter:
    __window: List
    __window_size: int

    def __init__(self, window_size: int, data_len: int):

        self.__window = []
        self.__window_size = window_size
        self.data_len = data_len
        self.value = 0

    def add_prediction(self, data) -> np.ndarray:
        self.__window.insert(0, data)
        self.__window = self.__window[:self.__window_size]

        temp_sum = np.zeros(self.data_len)
        for i, action in enumerate(self.__window):
            # Increase count 
            temp_sum[action] += 1 + i*0.2
            i += 1
        self.value = np.argmax(temp_sum)

    def get_value(self):
        return self.value
