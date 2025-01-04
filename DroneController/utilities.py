from enum import Enum
import yaml

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


class DynamicAttributes:
    def __init__(self, attributes_dict):
        for key, value in attributes_dict.items():
            setattr(self, key, value)

class ConfigParser:
    def __init__(self, param_file) -> None:
        with open(param_file) as file:
            param = yaml.safe_load(file)
        
        self.GUI = DynamicAttributes(param["GUI"])
        self.Trajectory = DynamicAttributes(param["Trajectory"])
        self.Vicon = DynamicAttributes(param["Vicon"])
        self.Drone = DynamicAttributes(param["Drone"])