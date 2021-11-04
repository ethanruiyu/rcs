import json
from datetime import datetime
from rcs.common.enum import *
from rcs.common.types import Pos
from ..common.utils.atomic_counter import AtomicCounter
from datetime import datetime
from threading import Event


MESSAGE_ID = AtomicCounter()

class Command:
    def __init__(self):
        self._message_id = MESSAGE_ID
        self._timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self._message_type = 0
        self._data = ''

    @property
    def message_id(self):
        return self._message_id.increment()

    def __json__(self):
        return json.dumps(self.__dict__())

    def __dict__(self):
        return {
            'timestamp': self._timestamp,
            'messageId': self.message_id,
            'messageType': self._message_type,
            'data': self._data
        }

    def __repr__(self):
        return self.__dict__()


class Heartbeat(Command):
    topic = '/root/{0}/heartbeat/set'

    def __init__(self):
        super().__init__()
        self._message_type = -1


class InitPosition(Command):
    topic = '/root/{0}/cmd/navigation/set'
    event = Event()
    def __init__(self, position: Pos):
        super(InitPosition, self).__init__()
        self._message_type = CommandEnum.INIT_POSITION.value
        self._data = position

    def __json__(self):
        return json.dumps({
            'timestamp': self._timestamp,
            'messageId': self.message_id,
            'messageType': self._message_type,
            'data': self._data
        })


class Drive(Command):
    topic = '/root/{0}/cmd/chassis/set'
    def __init__(self, speed):
        super(Drive, self).__init__()
        self._message_type = CommandEnum.DRIVE.value
        self._data = speed

    def __json__(self):
        return json.dumps({
            'timestamp': self._timestamp,
            'messageId': self.message_id,
            'messageType': self._message_type,
            'data': self._data
        })