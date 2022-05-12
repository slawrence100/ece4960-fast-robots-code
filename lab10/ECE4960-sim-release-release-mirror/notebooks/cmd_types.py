from enum import Enum

class CMD(Enum):
    SET_MOTOR_CALIB = 0
    SET_PID = 1
    OBSERVE = 2
    GET_DATA = 3
    MOVE_DISTANCE = 4
    TURN_DEGREES = 5

