from abc import ABC, abstractmethod

from proxemics_analizer.ProxemicAnalysisResult import ProxemicAnalysisResult


class ProxemicAnalysisHandler(ABC):
    """
    An interface describing an object that handles the processing result of a proxemic analyzer
    """

    @abstractmethod
    def handle_proxemic_data(self, result: ProxemicAnalysisResult) -> None:
        pass
