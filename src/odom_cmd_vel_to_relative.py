#!/usr/bin/env python

import rospy
from tools import tools_cmd_vel
from nav_msgs.msg import Odometry
import numpy as np
import math

i = 0
angle = 0

def callback_odom(msg):
    global i
    global angle

    orientation = msg.pose.pose.orientation
    angle = 2 * math.acos(orientation.w)
    degree = ((angle/(math.pi*2))*360.0)
    angle = degree*(math.pi/180)
    # if i<5:
    #     print(angle)
    
    if i%30==0:
        print(round(degree,1))
    i+=1

if __name__ == "__main__":
    rospy.init_node('print_tf')
    print(math.pi)

    odom_sub = rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.spin()
    try:
        x_goal = 1
        y_goal = 0
        goal = np.array([[x_goal],[y_goal]])
        rospy.sleep(1)
        rotation_matrix = np.array([[math.cos(angle), -math.sin(angle)], 
                [math.sin(angle), math.cos(angle)]])
        print (rotation_matrix)
        print(angle)
        r_goal = np.matmul(rotation_matrix, goal)
        print('new goal', r_goal)
        tools_cmd_vel.move_relative(float(r_goal[0][0]), float(r_goal[1][0]), duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")


