from enum import Enum

class CMD(Enum):
    SET_MOTOR_CALIB = 0
    SET_PID = 1
    MOVE_DISTANCE = 2
    TURN_DEGREES = 3
    STOP = 4
    DO_STUNT = 5
    GET_DATA = 6
    SPIN = 7
    MOVE_DURATION = 8

