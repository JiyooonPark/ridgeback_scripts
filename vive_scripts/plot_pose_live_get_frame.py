#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

graph_limit = 3

linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = [],[],[],[],[],[]
pose0, pose1, pose2 = [], [], []
real_x, real_y, real_z = [], [], []
position_x, position_y, position_z = [], [], []
T_M = []
R_M = []
i=0

def get_current_position(msg):

    global linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, i
    global pose0, pose1, pose2 
    global position_x, position_y, position_z

    # follows the conventional x, y, poses
    pose_position = msg.pose.pose.position
    position_x = pose_position.x
    position_y = pose_position.y
    position_z = pose_position.z
    i += 1

    linear_x.append(position_x)
    linear_y.append(position_y)
    linear_z.append(position_z)
    
    if i<10:
        pose0 = np.array([position_x, position_y, position_z])
    elif i == 50:
        print('Move forward')    
    elif 50< i < 500:
        pose1 = np.array([position_x, position_y, position_z])
    elif i == 500:
        print('Move right')
    elif 500< i < 1000:
        pose2 = np.array([position_x, position_y, position_z])
    elif 1000<i<1003:
        print_results()
    elif i>1050:
        real_world()

def real_world():
    global real_x, real_y, real_z

    # T_M_i = np.linalg.inv(T_M)
    position = np.array([position_x, position_y, position_z])
    # print(f'TMi: {T_M_i}\nposition_x:{position_x}')
    # print(T_M_i.shape, position.shape)
    # position_r = np.dot(T_M_i, position )
    # real_x.append(position_r[0])
    # real_y.append(position_r[1])
    # real_z.append(position_r[2])

    # print(position_r)
    moved_vector = np.array(pose1)- np.array(pose0)

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
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.gca(projection='3d')

def animate(i):
    # ax.clear()
    ax.set_xlim(-graph_limit, graph_limit)
    ax.set_ylim(-graph_limit, graph_limit)
    ax.set_zlim(-graph_limit, graph_limit)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Tracker Pose')
    ax.plot3D(linear_x, linear_y, linear_z)
    ax.plot3D(real_x, real_y, real_z, 'black')
rospy.init_node('print_tracker_pose')
odom_sub = rospy.Subscriber('/vive/LHR_0B028308_pose', PoseWithCovarianceStamped, get_current_position)
#LHR_0B028308_pose

ani = animation.FuncAnimation(fig, animate, interval=100)

plt.show()
# rospy.spin()