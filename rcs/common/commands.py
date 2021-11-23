import json
from datetime import datetime
from rcs.common.enum import *
from rcs.common.types import Pos
from ..common.utils.atomic_counter import AtomicCounter
from datetime import datetime
from threading import Event
from .types import Point
from ..plugins.planner.module import move_to_position


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
        print(self.__dict__())
        return json.dumps(self.__dict__())


class Heartbeat():
    topic = '/root/{0}/heartbeat/set'

    def __init__(self):
        super().__init__()
        self.data = {
            "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }

    def __repr__(self) -> str:
        return json.dumps(self.data)


class InitPosition(Command):
    topic = '/root/{0}/cmd/navigation/set'

    def __init__(self, position):
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


class SwitchMap(Command):
    topic = '/root/{0}/setting/map/set'

    def __init__(self, map_name: str):
        super(SwitchMap, self).__init__()
        self._message_type = SettingEnum.SWITCH_MAP.value
        self._data = map_name


class Mission(Command):
    topic = '/root/{0}/cmd/navigation/set'

    def __init__(self, vehicle_current_position, width,  raw):
        super(Mission, self).__init__()
        self._message_type = CommandEnum.MISSION.value
        self._path = []
        start_position = Point(
            x=vehicle_current_position[0], y=vehicle_current_position[1])
        data = []
        for step in raw:
            if step['action'] == 'MOVE_TO_POSITION':
                target_position = Point(
                    x=step['parameters']['position'][0],
                    y=step['parameters']['position'][1])
                step_path = move_to_position(
                    start_position,
                    target_position,
                    width)
                start_position = target_position
                self._path.append(step_path)
                data.append({
                    'action': step['action'],
                    'parameters': {
                        "path": step_path,
                        "seconds": None
                    },
                    'isFinal': False
                })
            if step['action'] == 'WAIT':
                seconds = step['parameters']['seconds']
                data.append({
                    'action': step['action'],
                    'parameters': {
                        "path": None,
                        "seconds": seconds
                    },
                    'isFinal': False
                })
        data[-1]['isFinal'] = True
        self._data = data

    def get_path(self):
        return self._path


class Pause(Command):
    topic = '/root/{0}/cmd/navigation/set'

    def __init__(self, value):
        super(Pause, self).__init__()
        if value == 0:
            self._message_type = CommandEnum.PAUSE.value
        if value == 1:
            self._message_type = CommandEnum.CONTINUE.value
        self._data = None

class Abort(Command):
    topic = '/root/{0}/cmd/navigation/set'

    def __init__(self):
        super(Abort, self).__init__()
        self._message_type = CommandEnum.ABORT_MISSION.value
        self._data = None


class SwitchMode(Command):
    topic = '/root/{0}/cmd/navigation/set'

    def __init__(self, mode):
        super(SwitchMode, self).__init__()
        self._message_type = CommandEnum.SWITCH_MODE.value
        self._data = mode