from abc import ABC, abstractmethod
from proxemics_analizer.proxemic_zone.ProxemicZone import ProxemicZone


class ProxemicZoneComputer(ABC):

    @abstractmethod
    def compute_zone(self, actor_distance: float) -> int:
        pass
