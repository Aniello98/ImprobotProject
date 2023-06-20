from abc import ABC, abstractmethod


class ActorPositionComputer(ABC):

    @abstractmethod
    def compute_position(self, angle: float) -> int:
        pass
