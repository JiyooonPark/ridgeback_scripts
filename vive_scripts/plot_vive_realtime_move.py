#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# ============= VARIABLES ============== #
graph_limit = 3

# ========== GLOBAL VARIABLES ========== #
vive_i, i = 0, 0
vive_pose0, vive_pose1, vive_pose2 = [], [], []
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
    global vive_i, transform_vector

    vive_i+=1

    if vive_i<=test_time0:
        print('init pose')
        vive_pose0 = [position_x, position_y, position_z]
        vive_pose1 = [position_x, position_y, position_z]

    else:
        vive_pose1 = [position_x, position_y, position_z]
    
    if test_time1<vive_i<test_time2:
        print('move right')
        moved_vector = np.array(vive_pose1)- np.array(vive_pose0)

        get_RM(moved_vector)
        
    if vive_i == test_time2:
        print('starting to print real value')
        print(R_M)

    if vive_i > test_time2:
        world_frame = transform_RM(vive_pose1)
        linear_vx.append(world_frame[0])
        linear_vy.append(world_frame[1])
        linear_vz.append(world_frame[2])

def get_RM(vector):
    global R_M
    angle = angle_between(vector, np.array([1,0,0]))
    R_My, now = rotate_y(vector, angle)
    angle = angle_between(now, np.array([0,0,1]))
    R_Mz, now = rotate_z(now, angle)
    angle = angle_between(now, np.array([0,1,0]))
    R_Mx, now = rotate_x(now, angle)

    R_M = np.dot(R_Mz, R_My)
    R_M = np.dot(R_Mx, R_M)

def transform_RM(vector):
    return np.dot(R_M, vector)

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
    odom_sub = rospy.Subscriber('/vive/LHR_2FCD60B0_pose', PoseWithCovarianceStamped, get_current_position)

    ani = animation.FuncAnimation(fig, animate, interval=10)

    plt.show()