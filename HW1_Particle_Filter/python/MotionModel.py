import sys
import numpy as np
import math


class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here
        """
        self.alpha1 = 0.05
        self.alpha2 = 0.04
        self.alpha3 = 0.03
        self.alpha4 = 0.04

    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        """
        TODO : Add your code here
        """
        rot1 = math.atan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2]
        trans = math.hypot(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0])
        rot2 = u_t1[2] - u_t0[2] - rot1
        
        stddev_rot1 = math.hypot(self.alpha1 * rot1, self.alpha2 * trans)
        stddev_trans = self.alpha4 * math.hypot(rot1, rot2)
        stddev_trans = math.hypot(stddev_trans, self.alpha3 * trans)
        stddev_rot2 = math.hypot(self.alpha1 * rot2, self.alpha2 * trans)
        
        rot1_noisy = np.random.normal(loc=rot1, scale=stddev_rot1)
        trans_noisy = np.random.normal(loc=trans, scale=stddev_trans)
        rot2_noisy = np.random.normal(loc=rot2, scale=stddev_rot2)
        
        x_t1 = x_t0
        x_t1[2] += rot1_noisy
        x_t1 += np.array([trans_noisy * math.cos(x_t1[2]), trans_noisy * math.sin(x_t1[2]), rot2_noisy])
        return x_t1


if __name__ == "__main__":
    pass
