#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
i = 0


def callback_odom(msg):
    global i
    pose = msg.pose.pose.position

    i = i+1
    # if i % 30 == 0:
    print("x: {:.3f} y: {:.3f} w: {:.3f}".format(pose.x, pose.y, pose.z))


rospy.init_node('print_tracker')
odom_sub = rospy.Subscriber('/vive/LHR_0B028308_odom', Odometry, callback_odom)

rospy.spin()
