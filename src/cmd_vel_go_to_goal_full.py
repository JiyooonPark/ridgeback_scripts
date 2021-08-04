#!/usr/bin/env python

# integration
from go_to_goal.go_to_goal_basic import movebase_client
from go_to_goal.go_to_goal_holonomic import go_to_goal_holonomic
import rospy

# go_to_goal_full.py

if __name__ == "__main__":
    # init x, y : the location wanted when facing the wall
    x_init = 3
    y_init = -1

    # goal x,y : the final goal
    x_goal = 3
    y_goal = -3

    try:
        rospy.init_node('movebase_client_py')
        rospy.loginfo("go to initial position started!")
        result = movebase_client(x_init, y_init)
        # result = movebase_client(0,0)
        if result:
            rospy.loginfo("go to initial position done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")

    rospy.loginfo("go to goal position started!")
    go_to_goal_holonomic(x_goal, y_goal)
    rospy.loginfo("go to goal position done!")
