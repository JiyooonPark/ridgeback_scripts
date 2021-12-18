import numpy as np
import math
theta_rx = math.pi/2
theta_ry = 0
theta_rz = 0
sin_rx, cos_rx = np.sin(theta_rx), np.cos(theta_rx)
sin_ry, cos_ry = np.sin(theta_ry), np.cos(theta_ry)
sin_rz, cos_rz = np.sin(theta_rz), np.cos(theta_rz)
# get the rotation matrix on x axis
R_Mx = np.array([[1,      0,       0],
                    [0, cos_rx, sin_rx],
                    [0, -sin_rx,  cos_rx]])
# get the rotation matrix on y axis
R_My = np.array([[cos_ry, 0, -sin_ry],
                    [     0, 1,       0],
                    [sin_ry, 0,  cos_ry]])
# get the rotation matrix on z axis
R_Mz = np.array([[cos_rz, sin_rz, 0],
                    [-sin_rz,  cos_rz, 0],
                    [     0,       0, 1]])
# compute the full rotation matrix
R_M = np.dot(np.dot(R_Mx, R_My), R_Mz)

print(np.matmul(np.matrix([0,0,1]), R_M))