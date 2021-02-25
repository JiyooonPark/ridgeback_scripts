#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def callback(msg):
    # rospy.sleep(5)
    print('s')
    print(msg)

rospy.init_node('check_odometry_odam')
odom_sub = rospy.Subscriber('/cmd_vel', Twist, callback)

rospy.spin()