#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry


def print_pose(msg):
    print('x: ', msg.x, 'y: ', msg.y)


rospy.init_node('turtlebot_check_pose')
listener = rospy.Subscriber('/turtle1/pose', Pose, print_pose)
rospy.spin()
