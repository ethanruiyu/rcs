from datetime import datetime
from rcs.common.enum import *
from rcs.common.types import *


class Command:
    def __init__(self):
        self._message_id = 0
        self._timestamp = ''

    @property
    def message_id(self):
        return self.message_id

    @message_id.setter
    def message_id(self, value: int):
        self._message_id += value


class InitPosition(Command):
    def __init__(self, position: Pose):
        super(InitPosition, self).__init__()
        self._message_type = CMDEnum.INIT_POSITION
        self._data = position
