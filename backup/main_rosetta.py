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


class ViconClient(QObject):
    pose_signal = pyqtSignal(dict)

    def __init__(self, parent: QObject) -> None:
        super().__init__(parent)
        self.tracker = tools.ObjectTracker("192.168.10.2")
        self.obj_name = "djimini"
        self.vicon_timer = QTimer()
        self.state = State.VICON_CONNECTED if self.vicon_timer.timeout.connect(self.pose_callback) else State.VICON_DISCONNECTED
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
            self.state = State.VICON_CONNECTED if self.vicon_timer.timeout.connect(self.pose_callback) else State.VICON_DISCONNECTED
            time.sleep(1)
        logging.info('vicon connected ...')
        self.vicon_timer.start(10)
       

class djiDrone(QObject):
    updateFlightModeGUI = pyqtSignal(VehicleMode)

    def __init__(self, parent: QObject) -> None:
        super().__init__(parent)
        self.state = State.LAND
        self.default_cmd_vel = 0.4
        self.default_dt = 0.05
        self.traj_dt = 300

    def mavlink_connect(self, connection_string):
        self.vehicle = connect(connection_string, wait_ready=True)
        self.state = State.HOVER
        self.vehicle.mode = VehicleMode("GUIDED")
        # Display Flight Mode
        self.updateFlightModeGUI.emit(self.vehicle.mode)
        self.addObserverAndInit(
            'mode'
            , lambda vehicle, name, mode: self.updateFlightModeGUI.emit(mode))
    
    def addObserverAndInit(self, name, cb):
        """We go ahead and call our observer once at startup to get an initial value"""
        self.vehicle.add_attribute_listener(name, cb)

    

############### MAV LINK communication ##########################################################################
    def publish_cmd_vel(self, velocity_x, velocity_y, velocity_z):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        # send command to vehicle on x Hz cycle
        elapsed_time = 0.0
        while elapsed_time < duration:
            self.publish_cmd_vel(velocity_x, velocity_y, velocity_z)
            time.sleep(self.default_dt)
            elapsed_time += self.default_dt
############### Joystick communication ##########################################################################
    def vehicle_validation(self, function):
        if self.vehicle.mode == "GUIDED":
            logging.info('button clicked ', function.__name__)
            function()
    @pyqtSlot()
    def west_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving west is not possible")
            return
        @self.vehicle_validation
        def west_wrapped():
            self.send_ned_velocity(0, self.default_cmd_vel, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)
    @pyqtSlot()
    def east_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving east is not possible")
            return
        @self.vehicle_validation
        def east_wrapped():
            self.send_ned_velocity(0, -self.default_cmd_vel, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)
    
    @pyqtSlot()
    def north_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving north is not possible")
            return
        @self.vehicle_validation
        def north_wrapped():
            self.send_ned_velocity(-self.default_cmd_vel, 0, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)
    @pyqtSlot()
    def south_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving south is not possible")
            return
        @self.vehicle_validation
        def south_wrapped():
            self.send_ned_velocity(self.default_cmd_vel, 0, 0, 1)
            # self.send_ned_velocity(0, 0, 0, 1)

    @pyqtSlot()
    def rtl_click(self):
        if self.state == State.LAND or self.state == State.INITIALIZED:
            logging.warning("[Invalid request]: landing is not possible")
            return
        @self.vehicle_validation
        def rtl_wrapped():
            self.vehicle.mode = VehicleMode("LAND")
            self.state = State.LAND

    @pyqtSlot()
    def up_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving up is not possible")
            return
        @self.vehicle_validation
        def up_wrapped():
            alt = self.vehicle.location.global_relative_frame.alt
            if alt < 3:
                self.send_ned_velocity(0, 0, -0.5 * self.default_cmd_vel, 1)
                # self.send_ned_velocity(0, 0, 0, 1)

    @pyqtSlot()
    def down_click(self):
        if self.state != State.HOVER:
            logging.warning("[Invalid request]: moving down is not possible")
            return
        @self.vehicle_validation
        def down_wrapped():
            alt = self.vehicle.location.global_relative_frame.alt
            if alt > 0.5:
                self.send_ned_velocity(0, 0, 0.5 * self.default_cmd_vel, 1)
                # self.send_ned_velocity(0, 0, 0, 1)

    def traj_callback(self):
        if self.state != State.HOVER:
            logging.info("trajectory timer canceled")
            self.traj_timer.stop()

        self.traj_index += 1
        if self.traj_index < len(self.traj):
            target = self.traj[self.traj_index][1:]
            vel = target - self.coord
            vx, vy, vz = vel
            logging.debug(f"[Traj] vx = {vx:.3f}, vy = {vy:.3f} ")
            self.publish_cmd_vel(-vy, -vx, 0.0)


        elif self.traj_index - 10 < len(self.traj):
            self.publish_cmd_vel(0, 0, 0)
        else:
            logging.debug("trajectory timer stopped")
            self.traj_timer.stop()

    @pyqtSlot()
    def sendTrajectory(self):

        if not os.path.exists(self.config.traj_path):
            logging.error(f'{self.config.traj_path} does not exist!')
            return
        logging.info('sending trajectory')

        if self.state == State.HOVER:
            self.traj = np.loadtxt(self.config.traj_path, delimiter=",")
            self.traj_timer = QTimer()
            self.traj_index = 0
            self.traj_timer.timeout.connect(self.traj_callback)
            self.traj_timer.start(self.traj_dt)




    
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi('window.ui', self)
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
        self.btnSendTraj.clicked.connect(self.dji.sendTrajectory)

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


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())