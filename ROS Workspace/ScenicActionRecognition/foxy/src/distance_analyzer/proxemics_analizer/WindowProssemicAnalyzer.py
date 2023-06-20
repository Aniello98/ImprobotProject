from typing import List, Optional

import numpy as np
from common.functions import *

from proxemics_analizer.ProssemicAnalyzer import ProssemicAnalyzer
from proxemics_analizer.ProxemicAnalysisResult import ProxemicAnalysisResult
from proxemics_analizer.ProxemicData import ProxemicData
from proxemics_analizer.actor_position.ActorPositionCounter import ActorPositionCounter
from proxemics_analizer.actor_position.StandardActorPositionComputer import \
    StandardActorPositionComputer
from proxemics_analizer.proxemic_zone.ProxemicZoneCounter import ProxemicZoneCounter
from proxemics_analizer.proxemic_zone.StandardProxemicZoneComputer import \
    StandardProxemicZoneComputer
from proxemics_analizer.signal_trend_classification.DerivativeTrendClassifier import \
    DerivativeTrendClassifier
from proxemics_analizer.signal_trend_classification.SignalTrendClassifier import \
    SignalTrendClassifier, SignalTrend

from proxemics_analizer.AngleAlignFSM import AngleAlignFSM

TIME_WINDOW = 20


class WindowProssemicAnalyzer(ProssemicAnalyzer):
    # This holds the embedding of the proxemic zone that is also used in messages, it is null at the beginning
    __zone_embedding: Optional[int]
    __position_embedding: Optional[int]
    __zone_counter: ProxemicZoneCounter
    __position_counter: ActorPositionCounter
    __trend_classifier: SignalTrendClassifier

    def __init__(self, analysis_handler) -> None:
        super(WindowProssemicAnalyzer, self).__init__(analysis_handler)
        self.__zone_embedding = None
        self.__position_embedding = None
        # Add the module to compute the proxemic zone, using the standard way of computing proxemic zones
        self.__zone_counter = ProxemicZoneCounter(
            StandardProxemicZoneComputer())
        self.__position_counter = ActorPositionCounter(
            StandardActorPositionComputer())
        self.__trend_classifier = DerivativeTrendClassifier()
        self.__angle_align_fsm = AngleAlignFSM()

    @threaded
    def process_person_data(self, data: List[ProxemicData]) -> None:

        # Here we should compute the data.
        """
        - Modulus is fine, we just compute from the coordinates
        - angle as well
        """

        # Here I receive a window of data, from them, I should compute all the data

        if len(data) == 0:
            return
        window_coordinates = np.array([[d.x, d.y] for d in data])

        # Compute the sequence of modulus
        modulus_sequence = self.__compute_modulus(window_coordinates)
        # And the sequence of theta angles
        theta_sequence = self.__compute_theta(window_coordinates)

        # === === PROXEMIC ZONE === ===

        # If the proxemic zone is null, compute it
        if self.__zone_embedding is None:
            for distance in modulus_sequence:
                self.__zone_embedding = self.__zone_counter.update(distance)
        else:
            # Otherwise, update it using the last computed distance
            self.__zone_embedding = self.__zone_counter.update(
                modulus_sequence[-1])

        # === === ACTOR POSITION === ===

        if self.__position_embedding is None:
            for angle in theta_sequence:
                self.__position_embedding = self.__position_counter.update(
                    angle)
        else:
            self.__position_embedding = self.__position_counter.update(
                theta_sequence[-1])

        # === === APPROACH / RETREAT === ===

        # Then use the trend classifier to classify the trend of the actor distance over the current time window
        distance_trend, distance_velocity = self.__trend_classifier.classify_trend(
            modulus_sequence)
        # Here just map the int into what we'll put into the message
        proxemic_movement = distance_trend.value

        # === === ANGLE TREND (used for lateral linear_movement) === ===
        # First make the FSM align the sequence of angles
        aligned_theta_sequence = self.__angle_align_fsm.align_signal(theta_sequence)
        # And then classify the trend
        angle_trend, angle_velocity = self.__trend_classifier.classify_trend(
            aligned_theta_sequence)
        lateral_movement = angle_trend.value

        # Create the result object
        result: ProxemicAnalysisResult = ProxemicAnalysisResult(
            linear_movement=proxemic_movement,
            lateral_movement=lateral_movement,
            zone=self.__zone_embedding,
            position=self.__position_embedding,
            linear_speed=distance_velocity,
            angular_speed=angle_velocity)

        # And notify the handler
        self._handler.handle_proxemic_data(result=result)

    @staticmethod
    def __compute_modulus(coordinates: np.ndarray) -> np.ndarray:
        print(coordinates.shape)
        return np.sqrt(np.power(coordinates[:, 0], 2) + np.power(coordinates[:, 1], 2))

    @staticmethod
    def __compute_theta(coordinates: np.ndarray) -> np.ndarray:
        # Compute angle in degrees using the arctan2 function from numpy
        return np.rad2deg(
            np.arctan2(coordinates[:, 1], coordinates[:, 0]))
