# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode
from pymavlink import mavutil

from PyQt4 import QtCore, QtGui
from window import Ui_MainWindow
import time


class QDCWindow(QtGui.QMainWindow):

	def __init__(self):
		
		QtGui.QMainWindow.__init__(self)

		# UI created by QT Designer 
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)

		# default value = 5 m
		self.launchAlt = 5 

		#Set up option parsing to get connection string
		import argparse  
		parser = argparse.ArgumentParser(description='Tracks GPS position of your computer (Linux only). Connects to SITL on local PC by default.')
		parser.add_argument('--connect', help="vehicle connection target.")
		args = parser.parse_args()

		self.connection_string = args.connect
		self.sitl = None

		#Start SITL if no connection string specified
		if not self.connection_string:
			import dronekit_sitl
			self.sitl = dronekit_sitl.start_default()
			self.connection_string = self.sitl.connection_string()

		# Connect to the Vehicle
		print 'Connecting to vehicle on: %s' % self.connection_string
		self.vehicle = connect(self.connection_string, wait_ready=True)

		# Display Flight Mode
		self.updateFlightModeGUI(self.vehicle.mode)
		self.addObserverAndInit(
			'mode'
			,lambda vehicle,name,mode: self.updateFlightModeGUI(mode) ) 

		# Display Location Info
		self.updateLocationGUI(self.vehicle.location)
		self.addObserverAndInit(
			'location'
			,lambda vehicle, name, location: self.updateLocationGUI(location) )


	def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
			0b0000111111000111, # type_mask (only speeds enabled)
			0, 0, 0, # x, y, z positions (not used)
			velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
			0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

		#send command to vehicle on 1 Hz cycle
		for x in range(0,duration):
			self.vehicle.send_mavlink(msg)
			time.sleep(1)

	# set yaw from 0 to 359 / 0-north, 90-east, 180-south, 270-west
	def condition_yaw(heading, relative=False):
		if relative:
			is_relative = 1 #yaw relative to direction of travel
		else:
			is_relative = 0 #yaw is an absolute angle
		# create the CONDITION_YAW command using command_long_encode()
		msg = vehicle.message_factory.command_long_encode(
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
			0, #confirmation
			heading,    # param 1, yaw in degrees
			0,          # param 2, yaw speed deg/s
			1,          # param 3, direction -1 ccw, 1 cw
			is_relative, # param 4, relative offset 1, absolute angle 0
			0, 0, 0)    # param 5 ~ 7 not used
		# send command to vehicle
		vehicle.send_mavlink(msg)
 	
	def updateLocationGUI(self, location):
		self.ui.lblLongValue.setText(str(location.global_frame.lon))
		self.ui.lblLatValue.setText(str(location.global_frame.lat))
		self.ui.lblAltValue.setText(str(location.global_relative_frame.alt))
	
	def updateFlightModeGUI(self, value):
		index,mode = str(value).split(':')
		self.ui.lblFlightModeValue.setText(mode)
		
	def addObserverAndInit(self, name, cb):
		"""We go ahead and call our observer once at startup to get an initial value"""
		self.vehicle.add_attribute_listener(name, cb)


	def vehicle_validation(self, function):
		if self.vehicle.mode == "GUIDED":
			function()

	def west_click(self):
		@self.vehicle_validation
		def wrapped():
			self.send_ned_velocity(0,-1,0,1)
			self.send_ned_velocity(0,0,0,1)

	def east_click(self):
		@self.vehicle_validation
		def wrapped():
			self.send_ned_velocity(0,1,0,1)
			self.send_ned_velocity(0,0,0,1)

	def north_click(self):
		@self.vehicle_validation
		def wrapped():
			self.send_ned_velocity(1,0,0,1)
			self.send_ned_velocity(0,0,0,1)

	def south_click(self):
		@self.vehicle_validation
		def wrapped():
			self.send_ned_velocity(-1,0,0,1)
   			self.send_ned_velocity(0,0,0,1)

	def rtl_click(self):
		@self.vehicle_validation
		def wrapped():
			self.vehicle.mode = VehicleMode("RTL")

	def up_click(self):
		@self.vehicle_validation
		def wrapped():
			alt = self.vehicle.location.global_relative_frame.alt
			if  alt < 20:
				self.send_ned_velocity(0,0,-0.5,1)
	 	  		self.send_ned_velocity(0,0,0,1)

	def down_click(self):
		@self.vehicle_validation
		def wrapped():
			alt = self.vehicle.location.global_relative_frame.alt
			if alt > 3:
				self.send_ned_velocity(0,0,0.5,1)
	 	  		self.send_ned_velocity(0,0,0,1)

	def launch_click(self):
		"""
		Arms vehicle and fly to self.alt
		"""
		print "Basic pre-arm checks"
		# Don't let the user try to arm until autopilot is ready
		while not self.vehicle.is_armable:
			print " Waiting for vehicle to initialise..."
			time.sleep(1)

		print "Arming motors"
		# Copter should arm in GUIDED mode
		self.vehicle.mode = VehicleMode("GUIDED")
		self.vehicle.armed = True

		while not self.vehicle.armed:      
			print " Waiting for arming..."
			time.sleep(1)

		print "Taking off!"
		self.vehicle.simple_takeoff(self.launchAlt) # Take off to target altitude

		# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
		#  after Vehicle.simple_takeoff will execute immediately).
		while True:
			print " Altitude: ", self.vehicle.location.global_relative_frame.alt      
			if self.vehicle.location.global_relative_frame.alt>=self.launchAlt*0.95: #Trigger just below target alt.
				print "Reached target altitude"
				break
			time.sleep(1)

	#def keyPressEvent (self, eventQKeyEvent):
	#	key = eventQKeyEvent.key()
	#	if key == QtCore.Qt.Key_Left:
	#		print 'Left'
	#	elif key == QtCore.Qt.Key_Up:
	#		print 'Up'
	#	elif key == QtCore.Qt.Key_Right:
	#		print 'Right'
	#	elif key == QtCore.Qt.Key_Down:
	#		print 'Down'

	#def keyReleaseEvent(self, eventQKeyEvent):
	#	key = eventQKeyEvent.key()
	#	if key == QtCore.Qt.Key_Left:
	#		print 'Left Released'
	#	elif key == QtCore.Qt.Key_Up:
	#		print 'Up Released'
	#	elif key == QtCore.Qt.Key_Right:
	#		print 'Right Released'
	#	elif key == QtCore.Qt.Key_Down:
	#		print 'Down Released'

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = QDCWindow()
    MainWindow.show()
    sys.exit(app.exec_()) 