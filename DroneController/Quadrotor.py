"""
Class for plotting a quadrotor

Author: Daniel Ingram (daniel-s-ingram)
"""

from math import cos, sin
import numpy as np

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure


class Quadrotor:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, size=0.25, show_animation=True):
        self.p1 = np.array([size / 2, 0, 0, 1]).T
        self.p2 = np.array([-size / 2, 0, 0, 1]).T
        self.p3 = np.array([0, size / 2, 0, 1]).T
        self.p4 = np.array([0, -size / 2, 0, 1]).T

        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.show_animation = show_animation
        self.path = None 

        if self.show_animation:
            # plt.ion()
            # fig = plt.figure()
            # # for stopping simulation with the esc key.
            # fig.canvas.mpl_connect('key_release_event',
            #         lambda event: [exit(0) if event.key == 'escape' else None])
            self.fig = Figure()
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.canvas = FigureCanvasQTAgg(self.fig)


        self.update_pose(x, y, z, roll, pitch, yaw)
    
    def setPath(self, path:np.ndarray):
        self.path = path


    def update_pose(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        if self.show_animation:
            self.plot()

    def transformation_matrix(self):
        x = self.x
        y = self.y
        z = self.z
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw
        return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch)
              * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll), z]
             ])

    def plot(self):  # pragma: no cover
        T = self.transformation_matrix()

        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)

        # plt.cla()
        self.ax.cla()

        self.ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0]],
                     [p1_t[1], p2_t[1], p3_t[1], p4_t[1]],
                     [p1_t[2], p2_t[2], p3_t[2], p4_t[2]], 'k.')

        self.ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]],
                     [p1_t[2], p2_t[2]], 'r-')
        self.ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]],
                     [p3_t[2], p4_t[2]], 'g-')

        self.ax.plot(self.x_data, self.y_data, self.z_data, 'b:')

        if self.path is not None:
            self.ax.plot(self.path[:, 0], self.path[:, 1], np.ones(len(self.path)), 'r')

        self.ax.set_xlim(-3.0, 3.0)
        self.ax.set_ylim(-3.0, 3.0)
        self.ax.set_zlim(0, 3)

        # plt.pause(0.001)
        self.canvas.draw()

if __name__ == '__main__':
    quad = Quadrotor(size=0.5)
    alt = 0
    while True:
        alt += 0.1
        quad.update_pose(0,0,alt,0,0,0)
        quad.plot()
        if alt > 5.0:
            break