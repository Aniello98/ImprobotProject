from abc import ABC, abstractmethod
from typing import List

# These values make sense for angles expressed in degrees 
OFFSET = 360
THRESHOLD = 359

# TODO: Consider if to change, but maybe not, since difference is anyway 360°

class AlignState(ABC):

    @abstractmethod
    def align_data(self, data: float) -> float:
        pass

    @abstractmethod
    def update(self, distance: float):
        pass


class NeutralState(AlignState):

    def align_data(self, data: float) -> float:
        return data

    def update(self, distance: float):
        if distance >= THRESHOLD:
            return JumpUpState()
        elif distance <= -THRESHOLD:
            return JumpDownState()
        else:
            return self


class JumpUpState(AlignState):

    def align_data(self, data: float) -> float:
        return data - OFFSET

    def update(self, distance: float):
        if distance <= -THRESHOLD:
            return NeutralState()
        else:
            return self


class JumpDownState(AlignState):

    def align_data(self, data: float) -> float:
        return data + OFFSET

    def update(self, distance: float):
        if distance >= THRESHOLD:
            return NeutralState()
        else:
            return self


class AngleAlignFSM:
    __state: AlignState

    def __init__(self):
        self.__reset()

    def __reset(self):
        self.__state = NeutralState()

    def align_signal(self, signal: List[float]) -> List[float]:
        self.__reset()
        output = [signal[0]]
        # First one we add
        for i, d in enumerate(range(1, len(signal))):
            dist = signal[i] - signal[i - 1]
            self.__state = self.__state.update(dist)
            converted = self.__state.align_data(signal[i])
            output.append(converted)
        return output
