#!/usr/bin/env python2.7

import rospy
from tools import tools_cmd_vel
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math

# ========= GLOBAL VARIABLES =========== #
rad = 0
position_x = 0
position_y = 0


def callback_odom(msg):

    global rad, position_x, position_y

    pose_position = msg.pose.pose.position
    position_x = pose_position.x
    position_y = pose_position.y
    position_z = pose_position.z

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    if yaw<0:
        angle = -yaw*180/math.pi

    else:
        angle = 360-yaw*180/math.pi

    rad = math.radians(angle)
    
if __name__ == "__main__":

    rospy.init_node('fixed_goal_any_orientation')

    odom_sub = rospy.Subscriber ('/odom', Odometry, callback_odom)
    
    try:
        x_goal = 0
        y_goal = 0

        x_move = x_goal-position_x
        y_move = y_goal-position_y

        # print(f'GOAL: {x_goal},{y_goal} CURRENT: {round(pose_x,2)}{round(pose_y,2)}')

        print (x_move, y_move)
        goal = np.array([[x_move],[y_move]])

        rotation_matrix = np.array([[math.cos(rad), -math.sin(rad)], 
                [math.sin(rad), math.cos(rad)]])

        r_goal = np.matmul(rotation_matrix, goal)

        print(rad)
        print(rotation_matrix)
        print('new goal', r_goal)
    
        tools_cmd_vel.move_relative(float(r_goal[0][0]), float(r_goal[1][0]), duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")


