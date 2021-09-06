#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
i = 0


def callback_odom(msg):
    global i

    i = i+1
    if i % 50 == 0:
        print("x: {:.3f} y: {:.3f} w: {:.3f}".format(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w))


rospy.init_node('print_odom')
odom_sub = rospy.Subscriber('/odom', Odometry, callback_odom)

rospy.spin()
