#!/usr/bin/env python

# integration
from simple_navigation_py import movebase_client
from go_to_goal_cmd_vel import go_to_goal_holonomic
import rospy



if __name__=="__main__":

    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client(3, 3)
        # result = movebase_client(0,0)
        if result:
            rospy.loginfo("go to initial position done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")


    go_to_goal_holonomic(3, -3)