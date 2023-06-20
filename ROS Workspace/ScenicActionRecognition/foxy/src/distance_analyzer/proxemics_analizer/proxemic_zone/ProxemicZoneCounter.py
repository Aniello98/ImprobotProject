from abc import ABC, abstractmethod

from proxemics_analizer.proxemic_zone.ProxemicZoneComputer import ProxemicZoneComputer
from impro_msgs.msg import ProxemicZone

LOW_THRESHOLD = 2
HIGH_THRESHOLD = 4
DISTANCE_EPSILON = 0.01


class ZoneState(ABC):

    def __init__(self, zone_embedding: int):
        self.zone_embedding = zone_embedding

    @abstractmethod
    def on_zone(self, zone: int):
        pass

    def get_zone(self) -> int:
        return self.zone_embedding


class IntimateZone(ZoneState):

    def __init__(self):
        super(IntimateZone, self).__init__(0)
        self.__neutral_counter = 0
        self.__no_int_counter = 0

    def on_zone(self, zone: float):
        if zone == 0:
            return self
        elif zone == 1:
            self.__neutral_counter += 1
            if self.__neutral_counter > LOW_THRESHOLD:
                return NeutralZone()
        else:
            self.__no_int_counter += 1
            if self.__no_int_counter > HIGH_THRESHOLD:
                return NoIntZone()
        return self


class NeutralZone(ZoneState):

    def __init__(self):
        super(NeutralZone, self).__init__(1)
        self.__intimate_counter = 0
        self.__no_int_counter = 0

    def on_zone(self, zone: float):
        if zone == 1:
            return self
        elif zone == 0:
            self.__intimate_counter += 1
            if self.__intimate_counter > LOW_THRESHOLD:
                return IntimateZone()
        else:
            self.__no_int_counter += 1
            if self.__no_int_counter > LOW_THRESHOLD:
                return NoIntZone()
        return self


class NoIntZone(ZoneState):
    def __init__(self):
        super(NoIntZone, self).__init__(2)
        self.__neutral_counter = 0
        self.__intimate_counter = 0

    def on_zone(self, zone: float):
        if zone == 2:
            return self
        elif zone == 1:
            self.__neutral_counter += 1
            if self.__neutral_counter > LOW_THRESHOLD:
                return NeutralZone()
        else:
            self.__intimate_counter += 1
            if self.__intimate_counter > HIGH_THRESHOLD:
                return IntimateZone()
        return self


class ProxemicZoneCounter:
    """
    A class representing an object used to transition between states
    """

    __zone_state: ZoneState
    __zone_computer: ProxemicZoneComputer

    def __init__(self, zone_computer: ProxemicZoneComputer):
        self.__zone_state = NoIntZone()
        self.__zone_computer = zone_computer

    def update(self, actor_distance: float) -> int:
        # First compute the zone from the distance
        zone = self.__zone_computer.compute_zone(actor_distance)
        # Then propagate in the FSM
        self.__zone_state = self.__zone_state.on_zone(zone)
        # And return the resulting zone
        return self.__zone_state.get_zone()
