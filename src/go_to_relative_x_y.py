#!/usr/bin/env python3

import rospy
from tools_cmd_vel import *
if __name__ == "__main__":

    rospy.init_node('holonimoic_move_to_goal')
    move_relative(-1, -5)
    rospy.spin()
