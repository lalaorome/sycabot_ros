from math import ceil
import string
import time
import numpy as np
import matplotlib.pyplot as plt

class Gridworld() :

    def __init__(self, gridsize: int) :

        self.gridsize: int = gridsize
        self.init_map()
        self.action_dict = {
            'up': (0,1),
            'down': (0,-1),
            'left': (-1,0),
            'right': (1,0),
        } 

    def init_map(self) -> None:
        x = np.linspace(-2,-2+self.gridsize*(ceil(4./self.gridsize)), ceil(4./self.gridsize)+1)
        y = np.linspace(-4,-4+self.gridsize*(ceil(8./self.gridsize)), ceil(8./self.gridsize)+1)
        self.xv, self.yv = np.meshgrid(x,y, indexing='ij')
        print(self.xv.shape)
        return
    
    def get_centroid(self, rob_poses: np.array) -> tuple:
        dist_arr = np.sqrt((self.xv - rob_poses[0])**2 + (self.yv - rob_poses[1])**2)
        centroid_idx = np.unravel_index(np.argmin(dist_arr, axis=None), dist_arr.shape)
        print(centroid_idx)
        return centroid_idx
    
    def get_next_goal(self, centroid_idx: tuple, action: string):
        x = centroid_idx[0] + self.action_dict[action][0] 
        y = centroid_idx[1] + self.action_dict[action][1]
        print(x,y)
        goal = np.array([self.xv[x,y],self.yv[x,y]])
        return goal 

