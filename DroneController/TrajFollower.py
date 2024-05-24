import numpy as np 
import os 
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot
from .utilities import State, DynamicAttributes
from .Drone import djiDrone
import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='[TrajFollower %H:%M:%S]')


class TrajFollower(QObject):

    emergencyLand = pyqtSignal()
    def __init__(self, coord:np.array, config:DynamicAttributes, drone: djiDrone, parent: QObject) -> None:
        super().__init__(parent)
        self.__config = config
        self.traj_dt = self.__config.traj_dt
        self.coord = coord
        self.drone = drone
        self.traj_path = self.__config.traj_path
        self.emergencyLand.connect(self.drone.rtl_click)
        self.fence = self.__config.fence
        

    def checkWithinGeoFence(self):
        x, y = self.coord[0], self.coord[1]

        if x >= self.fence[0] and x <= self.fence[1]:
            if y >= self.fence[2] and y <= self.fence[3]:
                return True
        return False 


    def traj_callback(self):
        if self.drone.state == State.LAND:
            logging.info("trajectory timer canceled")
            self.traj_timer.stop()

        if not self.checkWithinGeoFence():
            self.drone.publish_cmd_vel(0, 0, 0)
            logging.info("geofence enforce emergency landing ..")
            self.emergencyLand.emit()
            return

        self.traj_index += 1
        if self.traj_index < len(self.traj):
            target = self.traj[self.traj_index][1:]
            vel = target - self.coord[:3]
            vx, vy, vz = vel
            logging.debug(f"[Traj] vx = {vx:.3f}, vy = {vy:.3f} ")
            self.drone.publish_cmd_vel(-vy, -vx, 0.0)


        elif self.traj_index - 10 < len(self.traj):
            self.drone.publish_cmd_vel(0, 0, 0)
        else:
            logging.info("trajectory timer stopped")
            self.traj_timer.stop()

    @pyqtSlot()
    def sendTrajectory(self):

        if not os.path.exists(self.traj_path):
            logging.error(f'{self.traj_path} does not exist!')
            return
        logging.info('sending trajectory')

        if self.drone.state == State.HOVER:
            self.traj = np.loadtxt(self.traj_path, delimiter=",")
            self.traj_timer = QTimer()
            self.traj_index = 0
            self.traj_timer.timeout.connect(self.traj_callback)
            self.traj_timer.start(self.traj_dt)