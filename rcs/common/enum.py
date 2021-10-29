from enum import Enum


class CommandEnum(Enum):
    INIT_POSITION = 0
    PAUSE = 1
    CONTINUE = 2
    MAPPING_START = 3
    MAPPING_STOP = 4
    MISSION = 5
    SWITCH_MODE = 6
    DRIVE = 100
