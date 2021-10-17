#!/usr/bin/env python

import rospy
from tools import tools_cmd_vel
from tf.msg import tfMessage
import numpy as np
import math

i = 0
angle = 0

def callback_tf(msg):
    global i
    global angle

    tf = msg.transforms[0].transform.rotation
    angle = 2 * math.acos(tf.z)
    degree = ((angle/(math.pi*2))*360.0)
    angle = degree*(math.pi/180) - math.pi
    
    # if i%60==0:
    #     if msg.transforms[0].child_frame_id=="base_link":
    #         print(angle)
    # i+=1

if __name__ == "__main__":
    rospy.init_node('print_tf')


    odom_sub = rospy.Subscriber('/tf', tfMessage, callback_tf)
    rospy.sleep(1)
    print(angle)
    rotation_matrix = np.array([[math.cos(angle), -math.sin(angle)], 
                    [math.sin(angle), math.cos(angle)]])
    print(rotation_matrix)
    b = np.array([[1],[0]])
    # print(np.matmul(a, b))

    # goal x,y : the final goal
    x_goal = -3
    y_goal = -3
    goal = np.array([[x_goal],[y_goal]])
    r_goal = np.matmul(rotation_matrix, goal)
    print('new goal', r_goal)

    try:
        # rospy.init_node('tf_goal')
        tools_cmd_vel.move_relative(float(r_goal[0][0]), float(r_goal[1][0]), duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")

