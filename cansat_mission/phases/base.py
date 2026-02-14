from abc import ABC, abstractmethod


class BasePhaseHandler(ABC):
    @abstractmethod
    def execute(self, controller, snapshot):
        raise NotImplementedError
