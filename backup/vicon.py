import time 
from queue import Queue
from threading import Thread
from pyvicon_datastream import tools
import numpy as np 


class ViconClient(Thread):

    def __init__(self, buffer:dict, OBJECT_NAME:str = "djimini", VICON_TRACKER_IP:str = "192.168.10.2") -> None:
        self.tracker = tools.ObjectTracker(VICON_TRACKER_IP)
        self.obj_name = OBJECT_NAME
        self.buffer = buffer
        Thread.__init__(self)
    
    def run(self):
        while(True):
            latency, frameno, position = self.tracker.get_position(self.obj_name)
            if position != []:
                xyz_position = position[0][2:5] # get x,y,z only
                orientation = position[0][7] # get rotation around z axis
                self.buffer = {"position": np.array(xyz_position) / 1000.0, "orientation" : orientation}
                
            time.sleep(0.01)



if __name__ == "__main__":
    pose = Queue()

    vicon = ViconClient(pose)
    vicon.start()

    while True:
        try:
            if not pose.empty():
                print(pose.get())
        except KeyboardInterrupt:
            break
