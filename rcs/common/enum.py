from enum import Enum


class CMDEnum(Enum):
    INIT_POSITION = 0
    PAUSE = 1
    CONTINUE = 2
    MAPPING_START = 3
    MAPPING_STOP = 4
    MISSION = 5
    SWITCH_MODE = 6
    DRIVE = 100


class VehicleState(Enum):
    OFFLINE = 'offline'
    IDLE = 'idle'
    BUSY = 'busy'
    PAUSED = 'paused'
    ERROR = 'error'
    CHARGING = 'charging'
