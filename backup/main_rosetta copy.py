from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time 

from PyQt5.QtWidgets import QApplication, QTableWidgetItem, QMainWindow
import sys
from PyQt5 import uic
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot
from threading import Thread
from pyvicon_datastream import tools
from queue import Queue

import configparser
from enum import Enum
from collections import defaultdict
import time
from DroneController.Quadrotor import Quadrotor
import numpy as np
import os
import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='%H:%M:%S')


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


class Action(Enum):
    IS_ARMABLE = 0
    IS_ARMED = 1
    HAS_ARRIVED = 2
    DISARMED = 3
    IS_SIM = 5

    


class FSM:
    def __init__(self) -> None:
        self.transition = defaultdict(dict)
        self.transition[State.INITIALIZED][Action.IS_ARMABLE] = State.ARMED
        self.transition[State.ARMED][Action.IS_ARMED] = State.TAKEOFF
        self.transition[State.TAKEOFF][Action.HAS_ARRIVED] = State.HOVER
        self.transition[State.LAND][Action.DISARMED] = State.INITIALIZED

        self.transition[State.INITIALIZED][Action.IS_SIM] = State.VISUALIZATION

        # self.transition[State.LAND][Action.VICON_CONNECTED] = State.POSE_ESTIMATOR
        # self.transition[State.POSE_ESTIMATOR][Action.VICON_CONNECTED] = State.VISUALIZATION





if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())