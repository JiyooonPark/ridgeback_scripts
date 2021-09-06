#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def callback_amcl(msg):
    # rospy.sleep(1)
    print("x: {:.3f} y: {:.3f} w: {:.3f}".format(
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w))


rospy.init_node('print_amcl_pose')
odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback_amcl)

rospy.spin()
