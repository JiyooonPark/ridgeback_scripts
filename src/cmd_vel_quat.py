#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import math

from tools import tools_cmd_vel

PI = math.pi
x = 0
y = 0
w = 0
i = 0
def callback_odom(msg):  # odom_sub = rospy.Subscriber('/odom', Odometry, callback)
    # follows the conventional x, y, poses
    global x, y, w, i
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w = msg.pose.pose.orientation.w
    angle = 2 * math.acos(w)
    i+=1
    if i%25 == 0:
        print(angle)

if __name__ == "__main__":

    rospy.init_node('cmd_vel_rotate')
    odom_sub = rospy.Subscriber(
        '/odometry/filtered', Odometry,callback_odom)
    tools_cmd_vel.turn_right(330)
    
    
