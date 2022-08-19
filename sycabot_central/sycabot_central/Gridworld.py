from math import ceil
import string
import time
import numpy as np
import matplotlib.pyplot as plt

class Gridworld() :
    """
    In charge of generating the grid world and giving the centroids of objects on this grid. 
    """
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
        '''
        Initialise the map with exact grid step defined by the gridsize.
        '''
        x = np.linspace(-1.5,-1.5+self.gridsize*(ceil(3./self.gridsize)), ceil(3./self.gridsize)+1)
        y = np.linspace(-3.5,-3.5+self.gridsize*(ceil(7./self.gridsize)), ceil(7./self.gridsize)+1)
        self.xv, self.yv = np.meshgrid(x,y, indexing='ij')
        return
    
    def get_centroid(self, position: np.array) -> tuple:
        '''
        Return the closest centroid from the given position
        '''
        # Make a distance matrix from the robot pose to all the centroids
        dist_arr = np.sqrt((self.xv - position[0])**2 + (self.yv - position[1])**2)
        #Get the closest centroid index
        centroid_idx = np.unravel_index(np.argmin(dist_arr, axis=None), dist_arr.shape)
        return centroid_idx
    
    def get_next_goal(self, centroid_idx: tuple, action: string):
        '''
        Get the next centroid based on the current centroid and the next action.
        '''
        x = centroid_idx[0] + self.action_dict[action][0] 
        y = centroid_idx[1] + self.action_dict[action][1]
        # Check if out of boundaries
        if x >= self.xv.shape[0] :
            x = centroid_idx[0]
        elif x < 0 :
            x = 0
        if y >= self.xv.shape[1] :
            y = centroid_idx[1]
        elif y < 0 :
            y = 0 

        goal = np.array([self.xv[x,y],self.yv[x,y]])
        return goal 

