"""module to help manage states"""
from enum import Enum

class RobotState(Enum):
    """enum containing the states of the robot"""
    SEEK = 0
    MEET = 1
    LINEUP = 2
    YEET = 3
    UNSTUCK = 4
    TEST = 9
    EMPTY = 10
