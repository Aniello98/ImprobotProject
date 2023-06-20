from abc import ABC, abstractmethod
from enum import Enum
from typing import Tuple


class SignalTrend(Enum):
    UP = 0
    STILL = 1
    DOWN = 2


class SignalTrendClassifier(ABC):

    @abstractmethod
    def classify_trend(self, signal) -> Tuple[SignalTrend, float]:
        pass
