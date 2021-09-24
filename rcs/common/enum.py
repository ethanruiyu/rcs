from enum import Enum


class VehicleState(Enum):
    OFFLINE = 'offline'
    IDLE = 'idle'
    BUSY = 'busy'
    PAUSED = 'paused'
    ERROR = 'error'
    CHARGING = 'charging'
