#!/usr/bin/env python3
import numpy as np
import math
graph_limit = 3

linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = [],[],[],[],[],[]
pose0, pose1, pose2 = np.array([ 1.49971449, -0.31798303, -0.29524547]),np.array([ 1.03813982, -0.32580185, -0.50409508]), np.array([ 1.25342631, -0.32641697, -0.98949438])
real_x, real_y, real_z = [], [], []
position_x, position_y, position_z = [], [], []
T_M = []
R_M = []

v0 = np.array([-0.91096829, -0.0154313,  -0.41218763])
v1 = np.array([ 0.40543586 ,-0.00115842 ,-0.91412276])

v0 = v0 / np.linalg.norm(v0)
v1 = v1 / np.linalg.norm(v1)
v2 = np.cross(v0, v1.T)



def real_world():

    theta_rx = angle_between(np.array([0,0,1]), v0)
    # theta_rx = math.pi*2 - theta_rx
    print('angle between v0 and x:', np.degrees(theta_rx))
    sin_rx, cos_rx = np.sin(theta_rx), np.cos(theta_rx)

    # get the rotation matrix on x axis
    R_Mx = np.array([[1,      0,       0],
                        [0, cos_rx, -sin_rx],
                        [0, sin_rx,  cos_rx]])

    after_x0 = np.dot(R_Mx.T,v0)
    after_x1 = np.dot(R_Mx.T,v1)
    after_x2 = np.dot(R_Mx.T,v2)
    plot_vector(after_x0, 'pink')
    plot_vector(after_x1, 'pink')
    plot_vector(after_x2, 'pink')

    theta_ry = angle_between(np.array([1,0,0]), after_x0)
    sin_ry, cos_ry = np.sin(theta_ry), np.cos(theta_ry)

    # get the rotation matrix on y axis
    R_My = np.array([[cos_ry, 0, -sin_ry],
                        [     0, 1,       0],
                        [sin_ry, 0,  cos_ry]])

    after_xy0 = np.dot(R_My,after_x0)
    after_xy1 = np.dot(R_My,after_x1)
    after_xy2 = np.dot(R_My,after_x2)
    # plot_vector(after_xy, 'blue')
    plot_vector(after_xy0, 'black')
    plot_vector(after_xy1, 'black')
    plot_vector(after_xy2, 'black')

    theta_rz = angle_between(np.array([0,0,1]), after_xy0)
    sin_rz, cos_rz = np.sin(theta_rz), np.cos(theta_rz)

    # get the rotation matrix on z axis
    R_Mz = np.array([[cos_rz, sin_rz, 0],
                        [-sin_rz,  cos_rz, 0],
                        [     0,       0, 1]])

    # compute the full rotation matrix
    R_M = np.dot(np.dot(R_Mx, R_My), R_Mz)

    position_r = np.dot(R_M, position)
    real_x.append(position_r[0])
    real_y.append(position_r[1])
    real_z.append(position_r[2])


def print_results():

    global R_M, T_M
    print(f'pose0: {pose0}, pose1: {pose1}, pose2: {pose2}')
    v0 = pose1 - pose0
    v1 = pose2 - pose1
    v2 = np.cross(v1, v0)
    print(f'v0: {v0}, v1: {v1}, v2: {v2}')
    # ax.plot3D([0, v0[0], 0, v1[0], 0, v2[0]], [0, v0[1], 0, v1[1], 0, v2[1]], [0, v0[2], 0, v1[2], 0, v2[2]], 'red')

    v0_hat = v0 / np.linalg.norm(v0)
    v1_hat = v1 / np.linalg.norm(v1)
    v2_hat = v2 / np.linalg.norm(v2)

    ax.plot3D([0, v0_hat[0]], [0, v0_hat[1]], [0, v0_hat[2]], 'red')
    ax.plot3D([0, v1_hat[0]], [0, v1_hat[1]], [0, v1_hat[2]], 'blue')
    ax.plot3D([0, v2_hat[0]], [0, v2_hat[1]], [0, v2_hat[2]], 'green')

    print(f'======\nunit vector: \nv0: {v0_hat}\nv1: {v1_hat}\nv2: {v2_hat}')
    T_M = np.array([v0_hat, v1_hat, v2_hat])

def angle_between(v1, v2):
    # angle between two vectors in radians
    unit_vector_1 = v1 / np.linalg.norm(v1)
    unit_vector_2 = v2 / np.linalg.norm(v2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    return np.arccos(dot_product)


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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
    print(after)
    plot_vector(after, 'blue')
    return after

def rotate_y(vector, angle):

    theta_rx = angle
    sin_ry, cos_ry = np.sin(theta_rx), np.cos(theta_rx)

    # get the rotation matrix on y axis
    R_My = np.array([[cos_ry, 0, -sin_ry],
                        [     0, 1,       0],
                        [sin_ry, 0,  cos_ry]])

    after = np.dot(R_My,vector)
    print(after)
    plot_vector(after, 'blue')
    return after

def rotate_z(vector, angle):

    theta_rx = angle
    sin_rz, cos_rz = np.sin(theta_rx), np.cos(theta_rx)

    # get the rotation matrix on z axis
    R_Mz = np.array([[cos_rz, sin_rz, 0],
                        [-sin_rz,  cos_rz, 0],
                        [     0,       0, 1]])

    after = np.dot(R_Mz,vector)
    print(after)
    plot_vector(after, 'blue')
    return after

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
    # plot_vector(1,2,1, 'black')
    # plot_vector(-1,-2,1, 'blue')




    # plot_vector(v0, 'red')
    # plot_vector(v1, 'blue')
    # plot_vector(v2, 'green')
    plot_vector([1,2,3], 'red')
    # plot_vector([1,-3,2], 'blue')
    # theta_rx = angle_between(np.array([0,0,1]), [1,2,3])
    # theta_rx = math.pi*2 - theta_rx
    # print('angle between v0 and x:', np.degrees(theta_rx))
    # sin_rx, cos_rx = np.sin(theta_rx), np.cos(theta_rx)

    # # get the rotation matrix on x axis
    # R_Mx = np.array([[1,      0,       0],
    #                     [0, cos_rx, -sin_rx],
    #                     [0, sin_rx,  cos_rx]])

    # after_x0 = np.dot(R_Mx, [1,2,3])
    # plot_vector(after_x0, 'green')
    # plot_vector(pose2, 'black')

    # print(np.degrees(angle_between(v1, v0)))

    # real_world()
    angle = angle_between([1,2,3], np.array([0,0,1]))
    now = rotate_x([1,2,3], angle)
    angle = angle_between(now, np.array([0,0,1]))
    now = rotate_y(now, angle)
    # print(np.degrees(angle_between(now, np.array([0,0,1]))))
    print(now)
    # rotate_x([1,2,3], np.radians(74.5))
    # rotate_x([1,2,3], np.radians(57.69))
    plt.show()  