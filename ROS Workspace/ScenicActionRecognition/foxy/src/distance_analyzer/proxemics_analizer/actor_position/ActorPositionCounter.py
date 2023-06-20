from abc import ABC, abstractmethod

from proxemics_analizer.actor_position.ActorPositionComputer import ActorPositionComputer
from proxemics_analizer.proxemic_zone.ProxemicZoneComputer import ProxemicZoneComputer
from impro_msgs.msg import ActorPosition

LOW_THRESHOLD = 5
HIGH_THRESHOLD = 10
DISTANCE_EPSILON = 0.01


class PositionState(ABC):

    def __init__(self, position_embedding: int):
        self.position_embedding = position_embedding

    @abstractmethod
    def on_position(self, position: int):
        pass

    def get_position(self) -> int:
        return self.position_embedding


class FrontPosition(PositionState):

    def __init__(self):
        super(FrontPosition, self).__init__(ActorPosition.FRONT)
        self.__left_counter = 0
        self.__right_counter = 0
        self.__behind_counter = 0

    def on_position(self, position: float):
        if position == ActorPosition.FRONT:
            return self
        elif position == ActorPosition.LEFT:
            self.__left_counter += 1
            if self.__left_counter > LOW_THRESHOLD:
                return LeftPosition()
        elif position == ActorPosition.RIGHT:
            self.__right_counter += 1
            if self.__right_counter > LOW_THRESHOLD:
                return RightPosition()
        else:
            self.__behind_counter += 1
            if self.__behind_counter > HIGH_THRESHOLD:
                return BehindPosition()
        return self


class LeftPosition(PositionState):

    def __init__(self):
        super(LeftPosition, self).__init__(ActorPosition.LEFT)
        self.__front_counter = 0
        self.__right_counter = 0
        self.__behind_counter = 0

    def on_position(self, position: float):
        if position == ActorPosition.LEFT:
            return self
        elif position == ActorPosition.FRONT:
            self.__front_counter += 1
            if self.__front_counter > LOW_THRESHOLD:
                return FrontPosition()
        elif position == ActorPosition.BEHIND:
            self.__behind_counter += 1
            if self.__behind_counter > LOW_THRESHOLD:
                return BehindPosition()
        else:
            self.__right_counter += 1
            if self.__right_counter > HIGH_THRESHOLD:
                return RightPosition()
        return self


class RightPosition(PositionState):

    def __init__(self):
        super(RightPosition, self).__init__(ActorPosition.RIGHT)
        self.__left_counter = 0
        self.__front_counter = 0
        self.__behind_counter = 0

    def on_position(self, position: float):
        if position == ActorPosition.RIGHT:
            return self
        elif position == ActorPosition.BEHIND:
            self.__behind_counter += 1
            if self.__behind_counter > LOW_THRESHOLD:
                return BehindPosition()
        elif position == ActorPosition.FRONT:
            self.__front_counter += 1
            if self.__front_counter > LOW_THRESHOLD:
                return FrontPosition()
        else:
            self.__left_counter += 1
            if self.__left_counter > HIGH_THRESHOLD:
                return LeftPosition()
        return self


class BehindPosition(PositionState):

    def __init__(self):
        super(BehindPosition, self).__init__(ActorPosition.BEHIND)
        self.__left_counter = 0
        self.__right_counter = 0
        self.__front_counter = 0

    def on_position(self, position: float):
        if position == ActorPosition.BEHIND:
            return self
        elif position == ActorPosition.LEFT:
            self.__left_counter += 1
            if self.__left_counter > LOW_THRESHOLD:
                return LeftPosition()
        elif position == ActorPosition.RIGHT:
            self.__right_counter += 1
            if self.__right_counter > LOW_THRESHOLD:
                return RightPosition()
        else:
            self.__front_counter += 1
            if self.__front_counter > HIGH_THRESHOLD:
                return BehindPosition()
        return self


class ActorPositionCounter:
    """
    A class representing an object used to transition between states
    """

    __position_state: PositionState
    __position_computer: ActorPositionComputer

    def __init__(self, zone_computer: ActorPositionComputer):
        self.__position_state = FrontPosition()
        self.__position_computer = zone_computer

    def update(self, actor_distance: float) -> int:
        # First compute the zone from the distance
        position = self.__position_computer.compute_position(actor_distance)
        # Then propagate in the FSM
        self.__position_state = self.__position_state.on_position(position)
        # And return the resulting zone
        return self.__position_state.get_position()
