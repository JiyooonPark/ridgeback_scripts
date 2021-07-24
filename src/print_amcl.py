#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


def callback_amcl(msg):
    # rospy.sleep(1)
    print("x: {:.3f} y: {:.3f} w: {:.3f}".format(
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w))


rospy.init_node('check_amcl_pose')
odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)

rospy.spin()
