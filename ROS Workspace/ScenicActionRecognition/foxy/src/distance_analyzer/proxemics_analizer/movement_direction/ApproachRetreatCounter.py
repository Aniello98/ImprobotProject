from abc import ABC, abstractmethod

from proxemics_analizer.ProxemicData import DistanceVariationType


import math

TRANSITION_THRESHOLD = 5
DISTANCE_EPSILON = 0.01


class DistanceState(ABC):

    @abstractmethod
    def on_delta(self, delta: float):
        pass

    @abstractmethod
    def get_variation_type(self) -> DistanceVariationType:
        pass


class ApproachState(DistanceState):

    def __init__(self):
        super(ApproachState, self).__init__()
        self.__retreat_counter = 0
        self.__still_counter = 0

    def on_delta(self, delta: float):
        if delta < -DISTANCE_EPSILON:
            return self
        elif delta > DISTANCE_EPSILON:
            self.__retreat_counter += 1
            if self.__retreat_counter > TRANSITION_THRESHOLD:
                return RetreatState()
        else:
            self.__still_counter += 1
            if self.__still_counter > TRANSITION_THRESHOLD:
                return StillState()
        return self

    def get_variation_type(self) -> DistanceVariationType:
        return DistanceVariationType.APPROACH


class RetreatState(DistanceState):

    def __init__(self):
        super(RetreatState, self).__init__()
        self.__approach_counter = 0
        self.__still_counter = 0

    def on_delta(self, delta: float):
        if delta > DISTANCE_EPSILON:
            return self
        elif delta < -DISTANCE_EPSILON:
            self.__approach_counter += 1
            if self.__approach_counter > TRANSITION_THRESHOLD:
                return ApproachState()
        else:
            self.__still_counter += 1
            if self.__still_counter > TRANSITION_THRESHOLD:
                return StillState()
        return self

    def get_variation_type(self) -> DistanceVariationType:
        return DistanceVariationType.RETREAT


class StillState(DistanceState):
    def __init__(self):
        super(StillState, self).__init__()
        self.__retreat_counter = 0
        self.__approach_counter = 0

    def on_delta(self, delta: float):
        if -DISTANCE_EPSILON < delta < DISTANCE_EPSILON:
            return self
        elif delta > DISTANCE_EPSILON:
            self.__retreat_counter += 1
            if self.__retreat_counter > TRANSITION_THRESHOLD:
                return RetreatState()
        elif delta < -DISTANCE_EPSILON:
            self.__approach_counter += 1
            if self.__approach_counter > TRANSITION_THRESHOLD:
                return ApproachState()
        return self

    def get_variation_type(self) -> DistanceVariationType:
        return DistanceVariationType.STILL


class ApproachRetreatCounter:
    """
    A class representing an object used to transition between states
    """

    __distance_state: DistanceState

    def __init__(self):
        self.__distance_state = StillState()

    def update(self, new_data: float, old_data: float) -> DistanceVariationType:
        if math.isnan(new_data) or math.isnan(old_data):
            # If we have a not detected person, do not consider the variation
            # TODO: Be sure it makes sense
            return self.__distance_state.get_variation_type()
        delta = new_data - old_data
        # TODO: See if to use a fuzzy system here
        # Update the state, passing the computed delta
        self.__distance_state = self.__distance_state.on_delta(delta)
        return self.__distance_state.get_variation_type()
