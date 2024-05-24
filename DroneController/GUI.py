from dronekit import VehicleMode

from PyQt5.QtWidgets import QMainWindow
from PyQt5 import uic
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot


import time
from .Quadrotor import Quadrotor
import numpy as np

from .ViconClient import ViconClient
from .Drone import djiDrone
from .TrajFollower import TrajFollower

import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='[GUI %H:%M:%S]')



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        ui_path = "/home/airlab/PyDev/droneController/window.ui"
        uic.loadUi(ui_path, self)
        self.coord = np.zeros(4)

         # add visualizer
        self.quad = Quadrotor(size=0.5)
        self.viewer.addWidget(self.quad.canvas)

        
        self.vicon = ViconClient(self)
        self.vicon.pose_signal.connect(self.pose_callback)

        # viz timer 
        self.viz_timer = QTimer()
        self.viz_timer.timeout.connect(self.updatePoseGUI)
        self.viz_timer.start(30)


        # establish mavlink connection 
        self.dji = djiDrone(self)
        self.dji.updateFlightModeGUI.connect(self.updateFlightModeGUI)

        self.dji.mavlink_connect("0.0.0.0:18990")
        self.btnWest.clicked.connect(self.dji.west_click)
        self.btnEast.clicked.connect(self.dji.east_click)
        self.btnNorth.clicked.connect(self.dji.north_click)
        self.btnSouth.clicked.connect(self.dji.south_click)
        self.btnRTL.clicked.connect(self.dji.rtl_click)
        self.btnUp.clicked.connect(self.dji.up_click)
        self.btnDown.clicked.connect(self.dji.down_click)


        # trajectory follower 
        traj_path = "/home/airlab/PyDev/droneController/trajs/spiral8.csv"
        self.traj = TrajFollower(self.coord, traj_path, self.dji, self)
        self.btnSendTraj.clicked.connect(self.traj.sendTrajectory)

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

    @pyqtSlot(VehicleMode)
    def updateFlightModeGUI(self, value):
        logging.info(f'flight mode change to {value}')
        index, mode = str(value).split(':')
        self.lblFlightModeValue.setText(mode)
        self.progressBar.setValue(100)

    def updatePoseGUI(self):
        heading = np.deg2rad(45) + self.coord[3]
        self.quad.update_pose( self.coord[0], self.coord[1], self.coord[2],0,0, heading)