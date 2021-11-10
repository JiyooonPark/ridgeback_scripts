#!/usr/bin/env python3
from tools import tools_cmd_vel
import rospy
from nav_msgs.msg import Odometry
import numpy as np

odom_x = 0
odom_y = 0
odom_z = 0

tracker_x = 0
tracker_y = 0
tracker_z = 0


def get_odom(msg):
    global odom_x, odom_y, odom_z
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y
    odom_z = msg.pose.pose.orientation.w


def get_tracker(msg):
    global tracker_x, tracker_y, tracker_z
    tracker_x = msg.pose.pose.position.x
    tracker_y = msg.pose.pose.position.y
    tracker_z = msg.pose.pose.position.z


if __name__ == '__main__':
    try:
        rospy.init_node('drawing_ridgeback')

        odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, get_odom)
        tracker_sub = rospy.Subscriber('/tracker', Odometry, get_tracker)

        odom_init = np.array([odom_x, odom_y, odom_z])
        tracker_init = np.array([tracker_x, tracker_y, tracker_z])
        tools_cmd_vel.move_relative(1, 0)
        odom_post = np.array([odom_x, odom_y, odom_z])
        tracker_post = np.array([tracker_x, tracker_y, tracker_z])
        odom_vector = odom_post - odom_init
        tracker_vector = tracker_post = tracker_init
        transformation = np.matmul(np.linalg.inx(tracker_vector), odom_vector)
        print(transformation)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
