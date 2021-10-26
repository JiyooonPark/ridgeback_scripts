#!/usr/bin/env python2.7

import rospy
from tools import tools_cmd_vel
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math

i = 0
angle = 0
rad = 0

def callback_odom(msg):
    global i, angle, rad

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    if yaw<0:
        angle = -yaw*180/math.pi
    else:
        angle = 360-yaw*180/math.pi
    # if i%30==0:
    #     print(f"angle : {round(angle, 2)}")
    i+=1
    rad = math.radians(angle)

if __name__ == "__main__":
    rospy.init_node('print_tf')

    odom_sub = rospy.Subscriber ('/odomerty/filtered', Odometry, callback_odom)
    
    try:
        x_goal = 1
        y_goal = 0
        goal = np.array([[x_goal],[y_goal]])
        rospy.sleep(1)
        rotation_matrix = np.array([[math.cos(rad), -math.sin(rad)], 
                [math.sin(rad), math.cos(rad)]])
        print (rotation_matrix)
        print(angle)
        r_goal = np.matmul(rotation_matrix, goal)
        print('new goal', r_goal)
        tools_cmd_vel.move_relative(float(r_goal[0][0]), float(r_goal[1][0]), duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")


