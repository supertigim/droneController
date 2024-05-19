from dronekit import connect, VehicleMode
from pymavlink import mavutil

from PyQt5.QtWidgets import QApplication, QTableWidgetItem, QMainWindow
import sys
from PyQt5 import uic
from PyQt5.QtCore import QTimer
from threading import Thread

import argparse
from enum import Enum
from collections import defaultdict
import time
from Quadrotor import Quadrotor

TAKEOFF_ALTITUDE = 5 # m
DT = 1 # sec
CMD_VEL = 1 # m/s

class State(Enum):
    INITIALIZED = 0
    ARMED = 1
    TAKEOFF = 2
    HOVER = 3

class Action(Enum):
    IS_ARMABLE = 0
    IS_ARMED = 1
    HAS_ARRIVED = 2


class MainWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        uic.loadUi('window.ui', self)
        self.progressBar.setValue(0)
        self.thread = Thread(target=self.connect, args=(args,))
        self.thread.start()

        self.transition = defaultdict(dict)
        self.transition[State.INITIALIZED][Action.IS_ARMABLE] = State.ARMED
        self.transition[State.ARMED][Action.IS_ARMED] = State.TAKEOFF
        self.transition[State.TAKEOFF][Action.HAS_ARRIVED] = State.HOVER

        self.launchAlt = TAKEOFF_ALTITUDE
        self.btnLaunch.clicked.connect(self.launch_click)

        self.btnWest.clicked.connect(self.west_click)
        self.btnEast.clicked.connect(self.east_click)
        self.btnNorth.clicked.connect(self.north_click)
        self.btnSouth.clicked.connect(self.south_click)
        self.btnRTL.clicked.connect(self.rtl_click)
        self.btnUp.clicked.connect(self.up_click)
        self.btnDown.clicked.connect(self.down_click)

        # add visualizer
        self.quad = Quadrotor(size=0.5)

        self.viewer.addWidget(self.quad.canvas)

    def connect(self, args):
        self.connection_string = args.connect
        self.sitl = None

        # Start SITL if no connection string specified
        if not self.connection_string:
            import dronekit_sitl
            self.sitl = dronekit_sitl.start_default()
            self.connection_string = self.sitl.connection_string()

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % self.connection_string)

        self.vehicle = connect(self.connection_string, wait_ready=True)
        self.progressBar.setValue(25)

        # Display Flight Mode
        self.updateFlightModeGUI(self.vehicle.mode)
        self.addObserverAndInit(
            'mode'
            , lambda vehicle, name, mode: self.updateFlightModeGUI(mode))

        # Display Location Info
        self.updateLocationGUI(self.vehicle.location)
        self.addObserverAndInit(
            'location'
            , lambda vehicle, name, location: self.updateLocationGUI(location))

        # change state
        self.state = State.INITIALIZED
        #

    def updateFlightModeGUI(self, value):
        print('flight mode change to ', value)
        index, mode = str(value).split(':')
        self.lblFlightModeValue.setText(mode)

    def updateLocationGUI(self, location):
        self.lblLongValue.setText(str(location.global_frame.lon))
        self.lblLatValue.setText(str(location.global_frame.lat))

        self.lblAltValue.setText(str(location.global_relative_frame.alt))
        # location.local_frame.
        x = location.local_frame.east
        y = location.local_frame.north
        if x is None or y is None:
            x,  y = 0, 0
        # print(x, y)
        self.quad.update_pose(x,y,location.global_relative_frame.alt,0,0,0)

    def addObserverAndInit(self, name, cb):
        """We go ahead and call our observer once at startup to get an initial value"""
        self.vehicle.add_attribute_listener(name, cb)

    def getAction(self, state:State):
        """get an action based on the state of the vehicle"""
        if state == State.INITIALIZED and self.vehicle.is_armable:
            return Action.IS_ARMABLE
        elif state == State.ARMED and self.vehicle.armed and self.vehicle.mode.name == 'GUIDED':
            return  Action.IS_ARMED
        elif state == State.TAKEOFF  and self.vehicle.location.global_relative_frame.alt >= self.launchAlt * 0.95:
            return Action.HAS_ARRIVED
    def timerCallback(self):
        """
        complete the launch process, update the state machine
        """
        action = self.getAction(self.state)
        if action is None:
            return

        self.state = self.transition[self.state][action]
        print(self.state.name, self.vehicle.system_status.state)

        if self.state == State.ARMED:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            self.progressBar.setValue(50)

        elif self.state == State.TAKEOFF:
            self.vehicle.simple_takeoff(self.launchAlt)
            self.progressBar.setValue(75)

        elif self.state == State.HOVER:
            print("vehicle reached to hovering position")
            self.timer.stop()
            self.progressBar.setValue(100)

            ############### MAV LINK communication ##########################################################################
    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on x Hz cycle
        self.vehicle.send_mavlink(msg)
        time.sleep(DT)


    ############### Joystick communication ##########################################################################
    def vehicle_validation(self, function):
        if self.vehicle.mode == "GUIDED":
            print('button clicked ', function.__name__)
            function()

    def west_click(self):
        @self.vehicle_validation
        def west_wrapped():
            self.send_ned_velocity(0, -CMD_VEL, 0, 1)
            self.send_ned_velocity(0, 0, 0, 1)

    def east_click(self):
        @self.vehicle_validation
        def east_wrapped():
            self.send_ned_velocity(0, CMD_VEL, 0, 1)
            self.send_ned_velocity(0, 0, 0, 1)

    def north_click(self):
        @self.vehicle_validation
        def north_wrapped():
            self.send_ned_velocity(CMD_VEL, 0, 0, 1)
            self.send_ned_velocity(0, 0, 0, 1)

    def south_click(self):
        @self.vehicle_validation
        def south_wrapped():
            self.send_ned_velocity(-CMD_VEL, 0, 0, 1)
            self.send_ned_velocity(0, 0, 0, 1)

    def rtl_click(self):
        @self.vehicle_validation
        def rtl_wrapped():
            self.vehicle.mode = VehicleMode("LAND")

    def up_click(self):
        @self.vehicle_validation
        def up_wrapped():
            alt = self.vehicle.location.global_relative_frame.alt
            if alt < 20:
                self.send_ned_velocity(0, 0, -0.5, 1)
                self.send_ned_velocity(0, 0, 0, 1)

    def down_click(self):
        @self.vehicle_validation
        def down_wrapped():
            alt = self.vehicle.location.global_relative_frame.alt
            if alt > 3:
                self.send_ned_velocity(0, 0, 0.5 * CMD_VEL, 1)
                self.send_ned_velocity(0, 0, 0, 1)


    def launch_click(self):
        """
        Arms vehicle and fly to self.alt
        """
        print('launch button pressed')
        if self.state == State.INITIALIZED:
            print('initializeing taking off ...')
            self.timer = QTimer()
            self.timer.timeout.connect(self.timerCallback)
            self.timer.start(1000)
        else:
            print("launch has already been initialized !")


if __name__ == '__main__':
    # Set up option parsing to get connection string
    parser = argparse.ArgumentParser(
        description='Tracks GPS position of your computer (Linux only). Connects to SITL on local PC by default.')
    parser.add_argument('--connect', help="vehicle connection target.")
    args = parser.parse_args()

    app = QApplication(sys.argv)
    window = MainWindow(args)
    window.show()
    sys.exit(app.exec())