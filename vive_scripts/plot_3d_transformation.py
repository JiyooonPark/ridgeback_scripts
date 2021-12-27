#!/usr/bin/env python3


import numpy as np

# ============= VARIABLES ============== #
graph_limit = 3

# ========== GLOBAL VARIABLES ========== #
vive_i, i = 0, 0
vive_pose0, vive_pose1 = [], []
transform_vector = 0
R_M  = []

linear_x, linear_y, linear_z= [],[],[]
linear_vx, linear_vy, linear_vz= [],[],[]

test_time0, test_time1, test_time2 = 10, 50, 300

# ============= FUNCTIONS ============== #

def get_current_position(msg):

    global linear_x, linear_y, linear_z, i

    pose_position = msg.pose.pose.position
    position_x = pose_position.x
    position_y = pose_position.y
    position_z = pose_position.z

    i += 1

    if i<= 150:
        linear_x.append(round(position_x,3))
        linear_y.append(round(position_y,3))
        linear_z.append(round(position_z,3))

    global vive_pose0, vive_pose1
    global vive_i, transform_vector, R_M 

    vive_i+=1

    if vive_i<=test_time0:
        vive_pose0 = [position_x, position_y, position_z]
        vive_pose1 = [position_x, position_y, position_z]

    else:
        vive_pose1 = [position_x, position_y, position_z]
    
    if test_time1<vive_i<test_time2:

        moved_vector = np.array(vive_pose1)- np.array(vive_pose0)

        theta_rx = angle_between(np.array([1,0,0]), moved_vector)
        sin_rx, cos_rx = np.sin(theta_rx), np.cos(theta_rx)

        # get the rotation matrix on x axis
        R_Mx = np.array([[1,      0,       0],
                            [0, cos_rx, sin_rx],
                            [0, -sin_rx,  cos_rx]])

        after_x = np.dot(R_Mx,moved_vector)
        theta_ry = angle_between(np.array([0,1,0]), after_x)
        sin_ry, cos_ry = np.sin(theta_ry), np.cos(theta_ry)

        # get the rotation matrix on y axis
        R_My = np.array([[cos_ry, 0, -sin_ry],
                            [     0, 1,       0],
                            [sin_ry, 0,  cos_ry]])

        after_xy = np.dot(R_My,after_x)
        theta_rz = angle_between(np.array([0,0,1]), after_xy)
        sin_rz, cos_rz = np.sin(theta_rz), np.cos(theta_rz)

        # get the rotation matrix on z axis
        R_Mz = np.array([[cos_rz, sin_rz, 0],
                            [-sin_rz,  cos_rz, 0],
                            [     0,       0, 1]])

        # compute the full rotation matrix
        R_M = np.dot(np.dot(R_Mx, R_My), R_Mz)

        
    if vive_i == test_time2:
        print('starting to print real value')
        print(vive_pose0)
        print(vive_pose1)
        print(R_M)

    if vive_i > test_time2:
        world_frame = np.around(np.matmul(np.array(vive_pose1), R_M), decimals=3)
        linear_vx.append(round(world_frame[0],3))
        linear_vy.append(round(world_frame[1],3))
        linear_vz.append(round(world_frame[2],3))

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

def angle_between(v1, v2):
    # angle between two vectors in radians
    unit_vector_1 = v1 / np.linalg.norm(v1)
    unit_vector_2 = v2 / np.linalg.norm(v2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    return np.arccos(dot_product)

def animate(i):
    ax.clear()
    ax.set_xlim(-graph_limit, graph_limit)
    ax.set_ylim(-graph_limit, graph_limit)
    ax.set_zlim(-graph_limit, graph_limit)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')

    ax.set_title('Tracker Pose')

    ax.plot3D(linear_x, linear_y, linear_z, color='r')
    ax.plot3D(linear_vx, linear_vy, linear_vz, color='b')

if __name__=='__main__':

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    rospy.init_node('print_tracker_pose')
    odom_sub = rospy.Subscriber('/vive/LHR_0B028308_pose', PoseWithCovarianceStamped, get_current_position)

    ani = animation.FuncAnimation(fig, animate, interval=10)

    plt.show()