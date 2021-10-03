#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from cmd_vel_go_to_fixed_goal_movebase import movebase_go_to_goal

if __name__ == "__main__":

    try:
        rospy.init_node("movebase_client_py")
        odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
        x_points = [-1.65, -1.25, -0.88, -0.34, 0.13, 0.64, 1.19, 1.78, 2.0, 2.58, 3.05, 3.39, 3.96, 4.33, 4.74, 5.24, 5.86, 6.09, 6.36, 6.76, 7.29, 7.72, 8.21, 8.88, 9.02, 9.49, 9.96]
        y_points = [-4.75, -4.99, -5.19, -5.2, -5.11, -5.02, -5.0, -5.1, -5.16, -5.28, -5.25, -5.11, -4.76, -4.47, -4.37, -4.39, -4.76, -5.38, -5.53, -5.54, -5.41, -5.15, -5.02, -5.08, -5.14, -5.59, -5.75]
        y_positive = [abs(y)-1.5 for y in y_points]
        for (x, y) in zip(x_points, y_positive):

            result = movebase_go_to_goal(x + 1, y)
            
            if result:
                rospy.loginfo("Goal execution done!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")