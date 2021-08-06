from abc import ABC
from datetime import datetime


class BaseCommand(ABC):
    def __init__(self):
        self._message_id = 0

    @property
    def message_id(self):
        return self.message_id

    @message_id.setter
    def message_id(self, value: int):
        self._message_id += value


class InitPose(BaseCommand):
    def __init__(self, position, orientation):
        super(InitPose, self).__init__()
        self._position = position,
        self._orientation = orientation
        self._timestamp = datetime.now().timestamp()


class Pause(BaseCommand):
    def __init__(self):
        super(Pause, self).__init__()
        self._timestamp = datetime.now().timestamp()


class Continue(BaseCommand):
    def __init__(self):
        super(Continue, self).__init__()
        self._timestamp = datetime.now().timestamp()
