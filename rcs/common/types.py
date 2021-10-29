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
        self.x = x
        self.y = y
        self.z = z

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
        self.point = Point(pos[0][0], pos[0][1], pos[0][2])
        self.orientation = Quaternion(pos[1][0], pos[1][1], pos[1][2], pos[1][3])

    def __list__(self):
        return [self.point, self.orientation]

    def __repr__(self):
        return self.__list__()