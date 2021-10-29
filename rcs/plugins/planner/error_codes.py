from enum import Enum


class PlannerErrorCode(Enum):
    INIT_POSITION_IN_WALL = 'init_position_in_wall'
    GOAL_POSITION_IN_WALL = 'goal_position_in_wall'
