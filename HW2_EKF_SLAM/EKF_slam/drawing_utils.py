import numpy as np
import matplotlib
import matplotlib.pyplot as plt

'''
def draw_cov_ellipse(centroid, cov, edgecolor):
    
    eigvals, eigvec = np.linalg.eig(cov)
    cov_ellipse = matplotlib.patches.Ellipse(xy=centroid, \
                                             width=2 * np.sqrt(3 * eigvals[0]), \
                                             height=2 * np.sqrt(3 * eigvals[1]), \
                                             angle=np.rad2deg(np.arccos(eigvec[0, 0])), \
                                             edgecolor=edgecolor, \
                                             facecolor='none', \
                                             linewidth=1)
    ax = plt.gca()
    ax.add_patch(cov_ellipse)
'''

def draw_cov_ellipse(centroid, cov, edgecolor):
    
    U, s, _ = np.linalg.svd(cov)
    width = 2 * np.sqrt(9 * s[0])
    height = 2 * np.sqrt(9 * s[1])
    angle = np.rad2deg(np.arctan2(U[0, 1], U[0, 0]))
    cov_ellipse = matplotlib.patches.Ellipse(xy=centroid, \
                                             width=width, \
                                             height=height, \
                                             angle=angle, \
                                             edgecolor=edgecolor, \
                                             facecolor='none', \
                                             linewidth=0.5)
    ax = plt.gca()
    ax.add_patch(cov_ellipse)


def draw_traj_pred(x, cov):
    draw_cov_ellipse(x[0:2], cov[0:2, 0:2], edgecolor='violet')


def draw_traj_and_map(x_prev, x_curr, cov, t):

    color = 'blue'
    ax = plt.gca()
    
    ax.plot([x_prev[0], x_curr[0]], [x_prev[1], x_curr[1]], color, linewidth=0.5)
    ax.scatter(x_curr[0], x_curr[1], c=color, marker=(5, 2), linewidths=0.2)
    
    draw_cov_ellipse(x_curr[0:2], cov[0:2, 0:2], color)

    if t == 0:
        color = 'red'
    else:
        color = 'green'

    k = 3
    while k < len(x_curr):
        draw_cov_ellipse(x_curr[k:k + 2], cov[k:k + 2, k:k + 2], color)
        k = k + 2
    
