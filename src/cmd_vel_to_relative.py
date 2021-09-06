#!/usr/bin/env python3

import rospy
from tools import tools_cmd_vel

if __name__ == "__main__":

    # goal x,y : the final goal
    x_goal = 3
    y_goal = 3

    try:
        rospy.init_node('cmd_vel_go_to_goal')
        rospy.loginfo("started: cmd_vel_go_to_goal")
        tools_cmd_vel.move_relative(x_goal, y_goal, duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")

