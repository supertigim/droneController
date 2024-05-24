from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time 
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot


from .utilities import State

import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='[Drone %H:%M:%S]')


class djiDrone(QObject):
    updateFlightModeGUI = pyqtSignal(VehicleMode)

    def __init__(self, parent: QObject) -> None:
        super().__init__(parent)
        self.state = State.LAND
        self.default_cmd_vel = 0.4
        self.default_dt = 0.05
        

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



