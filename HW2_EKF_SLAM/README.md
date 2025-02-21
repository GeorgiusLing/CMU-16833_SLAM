# Extended Kalman Filter


This program implements extended Kalman Filter for the following scenario:

A robot is moving on the 2D ground plane. In each time step t, the robot is controlled to move forward (the x-direction of the robot’s coordinates) $d_t$ units, and then rotate $\alpha_t$ radian. The pose of the robot in the global coordinates at time t is written as a vector $p_t=[x_t \ y_t \ \theta_t]^T$, where $x_t$ and $y_t$ are the 2D coordinates of the robot’s position, and $\theta_t$ is the robot’s orientation. Assume the errors follow Gaussian distributions: $e_x \sim N(0, \sigma_x^2)$ in x-direction, $e_y \sim N(0, \sigma_y^2)$in y-direction, and $e_x \sim N(0, \sigma_\alpha^2)$ in rotation respectively (all in robot’s coordinates).

There are several landmarks on the map. Each landmark $l$ is observed by the robot at time $t$ with a laser sensor which gives a measurement of the bearing angle $\beta$ (in the interval $(−\pi, \pi ]$) and the range $r$, with noise $n_\beta \sim N(0, \sigma_\beta^2)$ and $n_r \sim N(0, \sigma_r^2)$ respectively.

The data folder contains the data file needed for this program. Each line represents either a control input or a series of measurements.
* Each line with 2 numerals represent a control input, with the format $d_t$ ,$\alpha_t$
* Each line with more numerals represents a series of measurements, with the format $r_1$, $\beta_1$, $r_2$, $\beta_2$ ...

The scripts folder contains 2 Python scripts. The `EKF_SLAM.py`is the main routine. The `drawing_utils.py`contains some functions for visualization.

![Image](https://github.com/GeorgiusLing/CMU-16833_SLAM/blob/master/HW2_EKF_SLAM/result/result.png?raw=true)