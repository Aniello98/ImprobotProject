from abc import ABC, abstractmethod
from typing import Dict, List, Deque, Optional

from collections import deque

from proxemics_analizer.ProxemicData import ProxemicData
from proxemics_analizer.ProxemicAnalysisHandler import ProxemicAnalysisHandler

from common.functions import threaded
from common.constants import PROX_TIME_WIN


import numpy as np
import time

TIME_DIFF_THRESH = 10
MAX_ACTOR_DIST = 4.5

STEP_DELAY = 0.5 # seconds


class ProssemicAnalyzer(ABC):
    """An abstract class representing an object used to describe the prossemic relationship
    between the actor and the robot
    """

    # Create a list of all the collected prossemic information
    _collected_info: Deque[ProxemicData]

    _handler: ProxemicAnalysisHandler

    def __init__(self, analysis_handler: ProxemicAnalysisHandler) -> None:
        super().__init__()
        self._collected_info = deque(maxlen=PROX_TIME_WIN)
        self._handler = analysis_handler
        self.__alive = True

        # Start the analysis thread
        self.analysis_timer()


    @threaded
    def analysis_timer(self) -> None:
        while(self.__alive):
            # Sleep
            time.sleep(STEP_DELAY)
            # Then get a copy of the data in the current window
            data_copy = list(self._collected_info)
            # Clear the buffer
            self._collected_info.clear()
            # And pass data to the analysis module
            self.process_person_data(data_copy)

    def on_no_person_detected(self) -> None:
        # Insert an empty measurement, at the max distance detectable
        empty_data = ProxemicData(0, MAX_ACTOR_DIST)
        self._collected_info.append(empty_data)

    def on_data_received(self, data: ProxemicData) -> None:
        """Notify the analyzer of the availability of a new data point

        Args:
            person_id: the identifier of the person the data refer to
            data: the object representing spatial info of the detected person
        """
        print(
            f"Distance is: {np.sqrt(np.power(data.x, 2) + np.power(data.y, 2))}")

        # Then add the new data point, the deque will automatically discard older values
        self._collected_info.append(data)

    @abstractmethod
    @threaded
    def process_person_data(self, data: List[ProxemicData]) -> None:
        """Process the data related to the specified person

        Args:
            person (int): the identifier of the person
            data (List[ProxemicData]): the list of data to process
        """
        pass

    def flush_collected_data(self) -> None:
        # Clear the buffer for every person
        self._collected_info.clear()