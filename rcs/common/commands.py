import json
from datetime import datetime
from rcs.common.enum import *
from rcs.common.types import *


class Command:
    def __init__(self):
        self._message_id = 0
        self._timestamp = ''

    @property
    def message_id(self):
        return self._message_id

    @message_id.setter
    def message_id(self, value: int):
        self._message_id += value


class InitPosition(Command):
    def __init__(self, position: Pose):
        super(InitPosition, self).__init__()
        self._message_type = CMDEnum.INIT_POSITION.value
        self._data = position

    def to_json(self):
        return json.dumps({
            'timestamp': self._timestamp,
            'messageId': self.message_id,
            'messageType': self._message_type,
            'data': self._data
        })


class Drive(Command):
    def __init__(self, speed):
        super(Drive, self).__init__()
        self._message_type = CMDEnum.DRIVE.value
        self._data = speed

    def to_json(self):
        return json.dumps({
            'timestamp': self._timestamp,
            'messageId': self.message_id,
            'messageType': self._message_type,
            'data': self._data
        })