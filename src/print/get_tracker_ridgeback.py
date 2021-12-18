#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

vive_i=0  
vive_pose0 = []
vive_pose1 = []
transform_vector = 0
R_M  = []


def get_vive_pose(msg):

    global vive_pose0, vive_pose1
    global vive_i, transform_vector, R_M 

    pose_position = msg.pose.pose.position
    position_x = pose_position.x
    position_y = pose_position.y
    position_z = pose_position.z

    vive_i+=1
    if vive_i<=10:
        vive_pose0 = [position_x, position_y, position_z]
        vive_pose1 = [position_x, position_y, position_z]
    else:
        vive_pose1 = [position_x, position_y, position_z]
    
    if 50<vive_i<150:
        moved_vector = np.array(vive_pose1)- np.array(vive_pose0)
        theta_rx = angle_between(np.array([1,0,0]), moved_vector)
        theta_ry = angle_between(np.array([0,1,0]), moved_vector)
        theta_rz = angle_between(np.array([0,0,1]), moved_vector)

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

        
    # if vive_i == 150:
    #     print('starting to print real value')
    #     print(vive_pose0)
    #     print(vive_pose1)
    #     print(R_M)
    # if vive_i >= 200:
    #     world_frame = np.around(np.matmul(np.array(vive_pose1), R_M), decimals=3)
    #     if vive_i%10 ==0:
    #         print(world_frame)


def angle_between(v1, v2):
    # rad
    unit_vector_1 = v1 / np.linalg.norm(v1)
    unit_vector_2 = v2 / np.linalg.norm(v2)
    # unit_vector_1 = np.squeeze(unit_vector_1)
    # unit_vector_2 = np.squeeze(unit_vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return angle

if __name__=="__main__":
    
    rospy.init_node('print_tracker_pose')
    vive_sub = rospy.Subscriber('/vive/LHR_0B028308_pose', PoseWithCovarianceStamped, get_vive_pose)
    
    rospy.spin()
