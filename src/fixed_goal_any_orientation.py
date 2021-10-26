#!/usr/bin/env python2.7

import rospy
from tools import tools_cmd_vel
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math

angle = 0
rad = 0
pose_x = 0
pose_y = 0


def callback_odom(msg):
    global angle, rad
    global pose_x, pose_y
    pose_x = msg.pose.pose.position.x + 1
    pose_y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    if yaw<0:
        angle = -yaw*180/math.pi
    else:
        angle = 360-yaw*180/math.pi
    rad = math.radians(angle)
    
if __name__ == "__main__":
    rospy.init_node('go_to_goal')

    odom_sub = rospy.Subscriber ('/odometry/filtered', Odometry, callback_odom)
    rospy.sleep(1)
    
    try:
        x_goal = 3
        y_goal = 2

        x_move = x_goal-pose_x
        y_move = y_goal-pose_y
        # print(f'GOAL: {x_goal},{y_goal} CURRENT: {round(pose_x,2)}{round(pose_y,2)}')
        
        goal = np.array([[x_move],[y_move]])

        rospy.sleep(1)
        rotation_matrix = np.array([[math.cos(rad), -math.sin(rad)], 
                [math.sin(rad), math.cos(rad)]])
        r_goal = np.matmul(rotation_matrix, goal)
        
        print('new goal', r_goal)
    
        tools_cmd_vel.move_relative(float(r_goal[0][0]), float(r_goal[1][0]), duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")


