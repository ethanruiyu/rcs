import logging
from abc import ABC, abstractmethod
from queue import Queue


class BasicVehicleCommAdapter(ABC):
    def __init__(self):
        self.logger = logging.getLogger()
        self.commandQueue = Queue()

    @abstractmethod
    def enable(self):
        pass

    @abstractmethod
    def is_enabled(self):
        pass

    @abstractmethod
    def disable(self):
        pass

    @abstractmethod
    def restart(self):
        pass

    @abstractmethod
    def enqueue(self, value):
        pass

    @abstractmethod
    def dequeue(self, value):
        pass

