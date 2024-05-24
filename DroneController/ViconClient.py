import time 
import numpy as np
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot
from pyvicon_datastream import tools
from .utilities import State

import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='[Vicon %H:%M:%S]')


class ViconClient(QObject):
    pose_signal = pyqtSignal(dict)

    def __init__(self, parent: QObject) -> None:
        super().__init__(parent)
        self.IP_ADDR = "192.168.10.2"
        self.tracker = tools.ObjectTracker(self.IP_ADDR)
        self.obj_name = "djimini"
        if self.tracker.is_connected:
            self.state = State.VICON_CONNECTED 
            logging.info('vicon connected ...')
        else:
            self.state = State.VICON_DISCONNECTED
            logging.error('vicon cannot be connected')
        self.connection_ressolve()



    def pose_callback(self):

        self.state = State.VICON_CONNECTED if self.tracker.is_connected else State.VICON_DISCONNECTED
        if self.state == State.VICON_DISCONNECTED:
            self.connection_ressolve()
            return

        latency, frameno, position = self.tracker.get_position(self.obj_name)
        if position != []:
            xyz_position = position[0][2:5] # get x,y,z only
            orientation = position[0][7] # get rotation around z axis
            data = {"position": np.array(xyz_position) / 1000.0, "orientation" : orientation}
            self.pose_signal.emit(data)

    def connection_ressolve(self):
        while self.state == State.VICON_DISCONNECTED:
            logging.warn('waiting for vicon to connect ...[!]')
            self.state = State.VICON_CONNECTED if self.tracker.connect(self.IP_ADDR) else State.VICON_DISCONNECTED
            time.sleep(1)
        logging.info('vicon connection ressolved ...')
        self.vicon_timer = QTimer()
        self.vicon_timer.timeout.connect(self.pose_callback)
        self.vicon_timer.start(1)
       

