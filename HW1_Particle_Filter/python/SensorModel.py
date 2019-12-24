import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm, expon, uniform
import pdb

from MapReader import MapReader

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        """
        TODO : Initialize Sensor Model parameters here
        """
        self.z_hit = 7
        self.z_short = 9
        self.z_max = 10
        self.z_rand = 11
        
        self.var_hit = 0.2
        self.rate_short = 7
        
        self.occupancy_map = occupancy_map
        
    def calc_likelihood(self, z_t1, z_t1_star):
        

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """

        """
        TODO : Add your code here
        """
        for index, z_t1 in enumerate(z_t1_arr):
            angle = x_t1[2] - math.pi / 2.0 + index * math.radians(index)
            

        return q    
 
if __name__=='__main__':
    pass