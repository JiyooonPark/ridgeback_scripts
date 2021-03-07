#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
    # rospy.sleep(1)
    print(msg.pose.pose.position)

rospy.init_node('check_amcl_pose')
odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)

rospy.spin()