#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def print_pose(msg):
    print(msg.pose.pose)


rospy.init_node('turtlebot_check_odometry')
listener = rospy.Subscriber('/odom', Odometry, print_pose)
rospy.spin()
