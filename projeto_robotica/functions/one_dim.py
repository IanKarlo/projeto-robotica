import numpy as np
import math

def transpose_matrix(m: np.ndarray):
    return m.transpose()

def inv_matrix(m: np.matrix):
    return np.linalg.inv(m)

def SE2_xy(x, y):
        
    homo_matrix = np.matrix([
        [1, 0, x],
        [0, 1, y],
        [0, 0, 1]
    ])

    return homo_matrix 

def SE2_theta(theta):
    theta_as_rad = theta
    theta_cos = math.cos(theta_as_rad)
    theta_sin = math.sin(theta_as_rad)

    homo_matrix = np.matrix([
        [theta_cos, -theta_sin, 0],
        [theta_sin, theta_cos, 0],
        [0, 0, 1]
    ])

    return homo_matrix