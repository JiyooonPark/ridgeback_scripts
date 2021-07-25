#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

from tools import *
from tool_callbacks import *
from tools_cmd_vel import *
from cylinder_laser_scan_distance import *
from cylinder_laser_scan_align import *

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

    angle_70 = average(msg.ranges, 70)
    angle_90 = average(msg.ranges, 90)
    angle_110 = average(msg.ranges, 110)


if __name__ == "__main__":

    rospy.init_node('cylinder_rotate')
    sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)
    while True:
        keep_align()
        keep_distance(1)

        a, c = angle_110, angle_90
        b = triangle(20, a, c)
        squeezed = squeeze_triangle(a, b, c)
        x, y = squeezed[2], squeezed[1]
        move_relative_rotate(x, y-0.15)
