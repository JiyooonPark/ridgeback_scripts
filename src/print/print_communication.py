#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray


def callback_amcl(msg):
    print(msg)


rospy.init_node('print_amcl_pose')
odom_sub = rospy.Subscriber('/iiwa_ridgeback_communicaiton/trajectory', PoseArray, callback_amcl)

rospy.spin()
