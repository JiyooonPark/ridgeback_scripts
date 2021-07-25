#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

import tools_etc
import tools_cylinder

'''
flow 
1. go to cylinder

repeat cycle 1
2. align itself
3. move right
4. turn 20', move x, move y

'''

angle_110 = 0
angle_70 = 0
angle_90 = 0


def callback_laser(msg):

    global angle_110, angle_70, angle_90

    angle_70 = tools_etc.average(msg.ranges, 70)
    angle_90 = tools_etc.average(msg.ranges, 90)
    angle_110 = tools_etc.average(msg.ranges, 110)


if __name__ == "__main__":

    rospy.init_node('cylinder_rotate')
    sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)
    i = 0
    while True:
        if angle_90 == 0:
            print('ignoring')
        else:

            tools_cylinder.convex_rotate(angle_90, angle_110)
        i += 1
