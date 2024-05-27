import numpy as np 
import os 
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, pyqtSlot
from .utilities import State, DynamicAttributes
from .Drone import djiDrone
from .EKF import ekf_estimation
import logging
logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    level=logging.INFO,
    datefmt='[TrajFollower %H:%M:%S]')



def generate_spiral_eight(num_points, x_scale=3.5, y_scale=2.0, t0=0.0):
    """
    Generates coordinates for points on a circle.

    Args:
        num_points (int): Number of points on the circle.
        x_scale (float, optional): Scaling factor for x-coordinate. Defaults to 2.0.
        y_scale (float, optional): Scaling factor for y-coordinate. Defaults to 2.0.
        t0 (float, optional): Offset value for the angle. Defaults to 0.0.

    Returns:
        tuple: A tuple containing two NumPy arrays (x_coordinates, y_coordinates).
    """
    theta = np.linspace(t0, t0 + 2*np.pi, num_points)  # Create angles for all points
    x = x_scale * np.cos(theta) * np.sin(theta)
    y = y_scale * np.sin(theta)
    return x, y

def calculate_derivaties(waypoints):
    """
    Calculates velocities for a given set of waypoints.

    Args:
        waypoints (list): A list of waypoints, where each waypoint is a tuple (x, y).

    Returns:
        list: A list of velocities, where each velocity is a tuple (vx, vy).
    """

    num_points = len(waypoints)
    velocities = []
    for i in range(1, num_points):
        # Get current and previous waypoints
        current_point = waypoints[i]
        prev_point = waypoints[i-1]

        # Calculate relative distance (assuming Euclidean distance)
        dx = current_point[0] - prev_point[0]
        dy = current_point[1] - prev_point[1]
        distance = np.sqrt(dx**2 + dy**2)
        if distance != 0.0:
            # Velocity is assumed to be constant between waypoints
            # (adjust this logic if you have additional information about speed)
            velocity = (dx / distance, dy / distance)
        else:
            velocity = (0.0, 0.0)

        velocities.append(velocity)
        velocity = (0.0, 0.0)
        velocities.append(velocity)

    return np.array(velocities)


class TrajFollower(QObject):

    emergencyLand = pyqtSignal()
    maxVelSignal = pyqtSignal(float)
    def __init__(self, coord:np.array, config:DynamicAttributes, drone: djiDrone, parent: QObject) -> None:
        super().__init__(parent)
        self.__config = config
        self.traj_dt = self.__config.traj_dt
        self.coord = coord
        self.drone = drone
        self.traj_path = self.__config.traj_path
        self.emergencyLand.connect(self.drone.rtl_click)
        self.fence = self.__config.fence
        self.num_points = 500
        self.traj = None 
        

    def checkWithinGeoFence(self):
        x, y = self.coord[0], self.coord[1]

        if x >= self.fence[0] and x <= self.fence[1]:
            if y >= self.fence[2] and y <= self.fence[3]:
                return True
        return False 
    
    
    def updateTrajPoints(self, num_points):
        self.num_points = num_points
        logging.info(f"[TrajFollower] update num of geom points {self.num_points}")


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
            # target = self.traj[self.traj_index][1:]
            # vel = target - self.coord[:3]
            # vx, vy, vz = vel

            target = self.traj[self.traj_index]
            target_vel = self.traj_vel[self.traj_index]
            target_acc = self.traj_acc[self.traj_index]
            dt = self.traj_dt / 1000.0

            setpoint = np.vstack((target, target_vel, target_acc)).flatten()
            current = np.squeeze(self.xEst)
            error = setpoint - current
            K = np.array([
                [1, 0, dt, 0, dt * dt / 2.0, 0],
                [0, 1, 0, dt, 0, dt * dt / 2.0]
            ])
            vx, vy = K @ error

            logging.debug(f"[Traj] vx = {vx:.3f}, vy = {vy:.3f} ")
            self.drone.publish_cmd_vel(-vy, -vx, 0.0)

            # update ekf 
            z = np.array([[self.coord[0]], [self.coord[1]]])
            u = np.array([[vx], [vy]])
            self.xEst, self.PEst = ekf_estimation(self.xEst, self.PEst, z, u, dt)
            self.VMAX = max(self.VMAX, np.sqrt(self.xEst[2, 0] ** 2 + self.xEst[3, 0] ** 2))
            logging.info(f"VMAX = {self.VMAX}")
            self.maxVelSignal.emit(self.VMAX)


        # elif self.traj_index - 10 < len(self.traj):
        #     self.drone.publish_cmd_vel(0, 0, 0)
        else:
            logging.info("trajectory timer stopped")
            self.traj_timer.stop()

    # @pyqtSlot()
    def sendTrajectory(self):

        # if not os.path.exists(self.traj_path):
        #     logging.error(f'{self.traj_path} does not exist!')
        #     return
        

        if self.drone.state != State.HOVER:
            return
        

        self.xEst = np.zeros((6, 1), dtype=np.float32)
        self.PEst = np.eye(6, dtype=np.float32)
        self.xEst[0, 0] = self.coord[0]
        self.xEst[1, 0] = self.coord[1]
        self.VMAX = 0.0


        # self.traj = np.loadtxt(self.traj_path, delimiter=",")

        # compute trajectory 
        xx, yy = generate_spiral_eight(num_points=self.num_points)
        self.traj = np.column_stack((xx, yy))
        self.traj_vel = calculate_derivaties(self.traj)
        self.traj_acc = calculate_derivaties(self.traj_vel)
        logging.info(f'sending trajectory points = {self.traj.shape}, vel = {self.traj_vel.shape}, acc = {self.traj_acc.shape}')



        self.traj_timer = QTimer()
        self.traj_index = 0
        self.traj_timer.timeout.connect(self.traj_callback)
        self.traj_timer.start(self.traj_dt)