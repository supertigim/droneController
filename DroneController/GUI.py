from dronekit import VehicleMode

from PyQt5.QtWidgets import QMainWindow
from PyQt5 import uic
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot


import time
from .Quadrotor import Quadrotor
import numpy as np
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot

from .ViconClient import ViconClient
from .Drone import djiDrone
from .TrajFollower import TrajFollower
from .utilities import ConfigParser

import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='[GUI %H:%M:%S]')



class MainWindow(QMainWindow):

    def __init__(self, config:ConfigParser):
        super().__init__()
        self.__config = config.GUI
        ui_path = self.__config.ui_path
        uic.loadUi(ui_path, self)
        self.coord = np.zeros(4) # x, y, z, yaw

         # add visualizer
        self.quad = Quadrotor(size=0.5)
        self.viewer.addWidget(self.quad.canvas)

        
        self.vicon = ViconClient(self, config.Vicon)
        self.vicon.pose_signal.connect(self.pose_callback)

        # viz timer 
        self.viz_timer = QTimer()
        self.viz_timer.timeout.connect(self.updatePoseGUI)
        self.viz_timer.start(self.__config.viz_dt)


        # establish mavlink connection 
        self.dji = djiDrone(self, config.Drone)
        self.dji.updateFlightModeGUI.connect(self.updateFlightModeGUI)

        self.btnWest.clicked.connect(self.dji.west_click)
        self.btnEast.clicked.connect(self.dji.east_click)
        self.btnNorth.clicked.connect(self.dji.north_click)
        self.btnSouth.clicked.connect(self.dji.south_click)
        self.btnRTL.clicked.connect(self.dji.rtl_click)
        self.btnUp.clicked.connect(self.dji.up_click)
        self.btnDown.clicked.connect(self.dji.down_click)


        # trajectory follower 
        self.traj = TrajFollower(self.coord, config.Trajectory, self.dji, self)
        self.btnSendTraj.clicked.connect(self.sendTrajectory)
        self.numPointsText.textChanged.connect(self.updateNumPoints)
        self.traj.maxVelSignal.connect(self.updateMaxVel)

    def sendTrajectory(self):
        self.traj.sendTrajectory()
        self.quad.setPath(self.traj.traj)
  

    def updateNumPoints(self):
        text = self.numPointsText.text()
        try:
            numPoints = int(text)
            self.traj.updateTrajPoints(numPoints)
        except:
            pass

    @pyqtSlot(dict)
    def pose_callback(self, data):
        posistion = data['position']
        yaw = data['orientation']

        x, y, z = posistion
        
        
        self.coord[0] = x
        self.coord[1] = y
        self.coord[2] = z
        self.coord[3] = yaw

        
        self.lblLongValue.setText(f"{x:.4f}")
        self.lblLatValue.setText(f"{y:.4f}")
        self.lblAltValue.setText(f"{z:.4f}")
        logging.debug(f'location: {x}, {y}, {z}')

    @pyqtSlot(str)
    def updateFlightModeGUI(self, value):
        logging.info(f'flight mode change to {value}')
        # index, mode = str(value).split(':')
        self.lblFlightModeValue.setText(value)
        self.progressBar.setValue(100)

    def updatePoseGUI(self):
        heading = np.deg2rad(45) + self.coord[3]
        self.quad.update_pose( self.coord[0], self.coord[1], self.coord[2],0,0, heading)

    @pyqtSlot(float)
    def updateMaxVel(self, value):
        self.lbMaxVal.setText(f"{value:.3f} m/s")