#!/usr/bin/env python3
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

graph_limit = 3

pose0, pose1, pose2 = np.array([ 1.49971449, -0.31798303, -0.29524547]),np.array([ 1.03813982, -0.32580185, -0.50409508]), np.array([ 1.25342631, -0.32641697, -0.98949438])
R_M = []

v0 = pose1 - pose0
v1 = pose2 - pose1

v0 = v0 / np.linalg.norm(v0)
v1 = v1 / np.linalg.norm(v1)
v2 = np.cross(v0, v1.T)

def angle_between(v1, v2):
    # angle between two vectors in radians
    unit_vector_1 = v1 / np.linalg.norm(v1)
    unit_vector_2 = v2 / np.linalg.norm(v2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    return np.arccos(dot_product)

def plot_vector(vector, color):
    ax.plot3D([0,vector[0]], [0,vector[1]], [0,vector[2]], color)

def rotate_x(vector, angle):

    theta_rx = angle
    sin_rx, cos_rx = np.sin(theta_rx), np.cos(theta_rx)

    # get the rotation matrix on x axis
    R_Mx = np.array([[1,      0,       0],
                    [0, cos_rx, -sin_rx],
                    [0, sin_rx,  cos_rx]])

    after = np.dot(R_Mx,vector)
    # plot_vector(after, 'lightcoral')
    return R_Mx, after

def rotate_y(vector, angle):

    theta_rx = angle
    sin_ry, cos_ry = np.sin(theta_rx), np.cos(theta_rx)

    # get the rotation matrix on y axis
    R_My = np.array([[cos_ry, 0, -sin_ry],
                        [     0, 1,       0],
                        [sin_ry, 0,  cos_ry]])

    after = np.dot(R_My,vector)
    # plot_vector(after, 'cyan')
    return R_My, after

def rotate_z(vector, angle):

    theta_rx = angle
    sin_rz, cos_rz = np.sin(theta_rx), np.cos(theta_rx)

    # get the rotation matrix on z axis
    R_Mz = np.array([[cos_rz, sin_rz, 0],
                        [-sin_rz,  cos_rz, 0],
                        [     0,       0, 1]])

    after = np.dot(R_Mz,vector)
    # plot_vector(after, 'lime')
    return R_Mz, after

if __name__=='__main__':

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    
    ax.set_xlim(-graph_limit, graph_limit)
    ax.set_ylim(-graph_limit, graph_limit)
    ax.set_zlim(-graph_limit, graph_limit)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Tracker Pose')

    plot_vector(v0, 'red')
    plot_vector(v1, 'blue')
    plot_vector(v2, 'green')

    print('v0:', v0)
    print('v1:', v1)
    print('v2:', v2)

    final_vector = []
    # for i in [v1, v2]:
    angle = angle_between(v1, np.array([1,0,0]))
    R_My, now = rotate_y(v1, angle)
    angle = angle_between(now, np.array([0,0,1]))
    R_Mz, now = rotate_z(now, angle)
    angle = angle_between(now, np.array([0,1,0]))
    R_Mx, now = rotate_x(now, angle)

    R_M = np.dot(R_Mz, R_My)
    R_M = np.dot(R_Mx, R_M)

    for i in [v0, v1, v2]:
        final_vector.append(np.dot(R_M, i))
        
    for i in final_vector:
        plot_vector(i, 'pink')

    plt.show()  