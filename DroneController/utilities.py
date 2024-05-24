from enum import Enum


class State(Enum):
    INITIALIZED = 0
    ARMED = 1
    TAKEOFF = 2
    HOVER = 3
    LAND = 4
    POSE_ESTIMATOR = 5
    VISUALIZATION = 6
    TRAJ_FOLLOWER = 7
    VICON_CONNECTED = 8
    VICON_DISCONNECTED = 9
