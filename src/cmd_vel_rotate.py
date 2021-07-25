#!/usr/bin/env python2.7

import rospy
from nav_msgs.msg import Odometry
import math

import tools_callbacks
import tools_cmd_vel

PI = math.pi

if __name__ == "__main__":

    rospy.init_node('holonimoic_move_to_goal')
    odom_sub = rospy.Subscriber(
        '/odom', Odometry, tools_callbacks.callback_odom)
    tools_cmd_vel.turn_right(2, PI)
