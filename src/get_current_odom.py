#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
i=0

def callback(msg):
    global i

    # rospy.sleep(5)
    # print('s')
    i=i+1
    if i % 50 == 0:
        print(msg.pose.pose.position)
    # else:
    #     continue

rospy.init_node('check_odometry_odom')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)

rospy.spin()