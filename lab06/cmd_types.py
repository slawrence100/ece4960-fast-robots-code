from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    ECHO = 3
    DANCE = 4
    SET_VEL = 5
    GET_FRONT_TOF = 6
    GET_REAR_TOF = 7
    GET_IMU = 8
    MOVE_FORWARD = 9
    STOP = 10
    START_DATA_COLLECTION = 11
    STOP_DATA_COLLECTION = 12
    GET_TOF1_DATA = 13
    START_PID = 14
    STOP_PID = 15
    GET_PID_DATA = 16
