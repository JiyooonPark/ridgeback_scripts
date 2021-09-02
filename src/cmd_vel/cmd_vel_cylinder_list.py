#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

import tools_callbacks
import tools_cmd_vel
import tools_etc
import cylinder_laser_scan_align
import cylinder_laser_scan_distance

'''
flow 

1. get 90, 100
2. calculate difference 
3. cmd_vel_holonomic_Relative_rotate

every stop check for distance and alignment

'''

angle_90 = 0
angle_100 = 0


if __name__ == "__main__":

    rospy.init_node('cylinder_rotate')
    sub = rospy.Subscriber('/front/scan', LaserScan,
                           tools_callbacks.detect_convex_concave)
    cylinder_laser_scan_align.keep_align()
    cylinder_laser_scan_distance.keep_distance(1)

    # while True:

    # for i in range(5):

    #     a, c = angle_110, angle_90
    #     b = triangle(20, a, c)
    #     squeezed = squeeze_triangle(a, b, c)
    #     x, y = squeezed[2], squeezed[1]
    #     move_relative_rotate(x-0.05, y-0.15)
    # detect_convex_concave(msg)
