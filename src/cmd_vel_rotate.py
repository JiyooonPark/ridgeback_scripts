#!/usr/bin/env python2.7

import rospy
from nav_msgs.msg import Odometry
import math

from tools import tools_callbacks
from tools import tools_cmd_vel

PI = math.pi

if __name__ == "__main__":

    rospy.init_node('cmd_vel_rotate')
    odom_sub = rospy.Subscriber(
        '/odometry/filtered', Odometry, tools_callbacks.callback_odom)
    tools_cmd_vel.turn_right(2, PI)
