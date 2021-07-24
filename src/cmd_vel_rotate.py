#!/usr/bin/env python2.7

import rospy
from nav_msgs.msg import Odometry
import time
import math

from tool_callbacks import *
from tools_cmd_vel import *

PI = math.pi

if __name__ == "__main__":

    rospy.init_node('holonimoic_move_to_goal')
    odom_sub = rospy.Subscriber('/odom', Odometry, callback_odom)
    turn_right(2, PI)
