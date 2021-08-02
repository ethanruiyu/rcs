from datetime import datetime


class InitPose:
    def __init__(self, position, orientation):
        self._position = position,
        self._orientation = orientation
        self._timestamp = datetime.now().timestamp()


class Pause:
    def __init__(self):
        self._timestamp = datetime.now().timestamp()


class Continue:
    def __int__(self):
        self._timestamp = datetime.now().timestamp()

