from typing import List


class VehicleState:
    OFFLINE = 'offline'
    IDLE = 'idle'
    BUSY = 'busy'
    PAUSE = 'pause'
    ERROR = 'error'
    CHARGING = 'charging'

    CHOICES = [
        (OFFLINE, 'offline'),
        (IDLE, 'idle'),
        (BUSY, 'busy'),
        (PAUSE, 'pause'),
        (ERROR, 'error'),
        (CHARGING, 'charging'),
    ]


class MissionState:
    RAW = 'raw'
    DISPATCH = 'dispatch'
    PROCESSED = 'processed'
    FINISHED = 'finished'
    PAUSED = 'paused'
    ABORT = 'abort'

    CHOICES = [
        (RAW, 'raw'),
        (DISPATCH, 'dispatch'),
        (PROCESSED, 'processed'),
        (FINISHED, 'finished'),
        (PAUSED, 'paused'),
        (ABORT, 'abort')
    ]


class Point:
    x: float = 0
    y: float = 0
    z: float = 0

    def __init__(self, x=0, y=0, z=0):
        self.x = round(x, 2)
        self.y = round(y, 2)
        self.z = round(z, 2)

    def __tuple__(self):
        return (self.x, self.y)

    def __list__(self):
        return [self.x, self.y, self.z]

    def __dict__(self):
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z
        }

    def __repr__(self) -> list:
        return self.__list__()


class Quaternion:
    x: float = 0
    y: float = 0
    z: float = 0
    w: float = 0

    def __init__(self, x=0, y=0, z=0, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __list__(self):
        return [self.x, self.y, self.z, self.w]

    def __dict__(self):
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'w': self.w
        }

    def __repr__(self) -> list:
        return self.__list__()


class Pos:
    point = Point()
    orientation = Quaternion()

    def __init__(self, pos: list):
        self.point = Point(round(pos[0], 2), round(pos[1], 2), round(pos[2], 2))
        self.orientation = Quaternion(round(pos[3], 2), round(pos[4], 2), round(pos[5], 2), round(pos[6], 2))

    def __list__(self):
        return [self.point.x,
                self.point.y,
                self.point.z,
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
                self.orientation.w
                ]

    def __repr__(self):
        return self.__list__()


class Vehicle:
    state = VehicleState.OFFLINE
    name: str = ''
    x: float = 0
    y: float = 0
    orientation: List[float] = [0, 0, 0, 1]
    online = False
    
    def __init__(self, vehicle_name: str) -> None:
        self.name = vehicle_name

    def set_position(self, position):
        self.x = position[0]
        self.y = position[1]
        self.orientation = [position[3], position[4], position[5], position[6] ]


class Mission:
    state = MissionState.RAW
    name: str = ''
    sn: str = ''

    def __init__(self, sn) -> None:
        self.sn = sn