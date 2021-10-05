#!/usr/bin/env python3

import rospy
from tools import tools_cmd_vel

if __name__ == "__main__":

    rospy.init_node('holonimoic_move_to_goal')
    tools_cmd_vel.move_relative(1, 5)
    # rospy.spin()
