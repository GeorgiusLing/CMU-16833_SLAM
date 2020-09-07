#========================================================
# 
#   16833 Robot Localization and Mapping  
#   Assignment #2 
#   EKF-SLAM       
# 
#========================================================

import numpy as np
import scipy.linalg
import scipy.spatial
import matplotlib.pyplot as plt

from drawing_utils import draw_traj_pred, draw_traj_and_map
from scipy.spatial.distance import mahalanobis

# TEST: Setup uncertainity parameters (try different values!)
stddev_x = 0.25
stddev_y = 0.1
stddev_alpha = 0.1
stddev_beta = 0.01
stddev_r = 0.08

# Open data file and read measurement data
with open('../data/data.txt') as file:
    lines = file.readlines()

line = lines.pop(0)
measure = [float(item) for item in line.split()]
t = 0

# Setup control and measurement covariances
control_cov = np.diag([stddev_x ** 2, stddev_y ** 2, stddev_alpha ** 2])
measure_cov = np.diag([stddev_beta ** 2, stddev_r ** 2])

# Setup initial pose vector and pose uncertainty
pose = np.zeros(3)
pose_cov = np.diag([0.02 ** 2, 0.02 ** 2, 0.1 ** 2])

# Setup initial landmark vector landmark[] and covariance matrix landmark_cov[]
landmark = np.empty_like(measure)
landmark_cov = np.empty((0, 0))

for k in range(len(measure) // 2):
    beta = measure[2 * k]
    r = measure[2 * k + 1]

    dx = r * np.cos(beta + pose[2])
    dy = r * np.sin(beta + pose[2])    
    
    landmark[2 * k : 2 * k + 2] = pose[0:2] + np.array([dx, dy])
    
    landmark_pose = np.array([[1, 0, -r * np.sin(pose[2] + beta)], [0, 1, r * np.cos(pose[2] + beta)]])
    landmark_measure = np.array([[-r * np.sin(pose[2] + beta), np.cos(pose[2] + beta)], [r * np.cos(pose[2] + beta), np.sin(pose[2] + beta)]])
    landmark_cov = scipy.linalg.block_diag(landmark_cov, np.matmul(np.matmul(landmark_pose, pose_cov), landmark_pose.T) \
                                           +np.matmul(np.matmul(landmark_measure, measure_cov), landmark_measure.T))

# Setup state vector x with pose and landmark vector
state_vec = np.hstack((pose, landmark))

# Setup estimate covariance matrix with pose and landmark covariances
estimate_cov = scipy.linalg.block_diag(pose_cov, landmark_cov)
measure_cov = np.kron(np.identity(len(measure) // 2), measure_cov)

# Plot initial state and covariance
state_vec_prev = np.empty_like(state_vec)
np.copyto(state_vec_prev, state_vec)
draw_traj_and_map(state_vec, state_vec_prev, estimate_cov, t)

# Read control data
while len(lines) > 0:
    # Iteration & read next control data
    t += 1
    line = lines.pop(0)
    d, alpha = [float(item) for item in line.split()]
    
    # Predict Step    
    control_vec = np.array([d, 0, alpha])
    control_jac = scipy.spatial.transform.Rotation.from_euler('z', state_vec[2], degrees=False).as_matrix()    

    state_vec_pred = np.array(state_vec)
    state_vec_pred[0:3] = state_vec_pred[0:3] + np.matmul(control_jac, control_vec)
    
    tx = d * np.cos(-state_vec[2])
    ty = d * np.sin(-state_vec[2])

    state_jac = np.array([[1, 0, ty], [0, 1, tx], [0, 0, 1]])
    state_jac = scipy.linalg.block_diag(state_jac, np.identity(len(measure)))
    
    estimate_cov_pred = np.matmul(np.matmul(state_jac, estimate_cov), state_jac.T)
    estimate_cov_pred[0:3, 0:3] = estimate_cov_pred[0:3, 0:3] + np.matmul(np.matmul(control_jac, control_cov), control_jac.T)
    
    # Draw predicted state and estimate covariance
    draw_traj_pred(state_vec_pred, estimate_cov_pred)
    
    # Read measurement data
    line = lines.pop(0)
    measure = [float(item) for item in line.split()]
    
    # Update Step
    measure_jac = np.empty((0, 3 + len(measure)))
    measure_pred = []
    
    for i in range(len(measure) // 2):
        
        dx, dy = state_vec_pred[3 + 2 * i : 5 + 2 * i] - state_vec_pred[0:2]

        r = np.hypot(dx, dy)
        beta = np.arctan2(dy, dx)
        
        measure_jac = np.vstack((measure_jac, np.zeros((2, 3 + len(measure)))))
        measure_jac[-2, 0:2] = [dy / r / r, -dx / r / r]
        measure_jac[-1, 0:2] = [-dx / r, -dy / r]
        measure_jac[-2:, 2] = [-1, 0]
        measure_jac[-2:, 3 + 2 * i : 5 + 2 * i] = -measure_jac[-2:, 0:2]

        measure_pred.append((beta - state_vec_pred[2] + np.pi) % (2 * np.pi) - np.pi)
        measure_pred.append(r)

    measure = np.array(measure)
    measure_pred = np.array(measure_pred)
    
    kalman_gain = estimate_cov_pred @ (measure_jac.T) @ scipy.linalg.inv(measure_jac @ estimate_cov_pred @ (measure_jac.T) + measure_cov)

    state_vec = state_vec_pred + kalman_gain @ (measure - measure_pred)
    estimate_cov = (np.identity(estimate_cov.shape[0]) - kalman_gain @ measure_jac) @ estimate_cov_pred
    
    # Plot
    
    draw_traj_and_map(state_vec_prev, state_vec, estimate_cov, t)
    state_vec_prev[:] = state_vec

plt.xlim(-10, 25)
plt.ylim(-5, 30)
plt.axis('equal')

# EVAL: Plot ground truth landmarks

ground_truth = np.array([[3, 6], [3, 12], [7, 8], [7, 14], [12, 6], [11, 12]])

for i in range(ground_truth.shape[0]):
    plt.scatter(ground_truth[i, 0], ground_truth[i, 1], c='black', marker=(5, 1), linewidths=2)
    print('Error for landmark' + str(i + 1))
    distance = scipy.spatial.distance.mahalanobis(ground_truth[i], state_vec[3 + 2 * i : 5 + 2 * i], scipy.linalg.inv(estimate_cov[3 + 2 * i : 5 + 2 * i, 3 + 2 * i : 5 + 2 * i]))
    print('Mahalanobis: ' + str(distance))
    distance = scipy.spatial.distance.euclidean(ground_truth[i], state_vec[3 + 2 * i : 5 + 2 * i])
    print('Euclidean: ' + str(distance))

plt.show()
