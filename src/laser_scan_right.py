#! /usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy
import math, time
from geometry_msgs.msg import Twist

from tools import *
from tools_cmd_vel import *

angle_90=0
angle_60=0
angle_120=0
m = 0

def diff(angle1, angle2):
    print("difference of : {:.4} and {:.4} is {:.4}".format(angle1, angle2, angle1-angle2))
    return angle1-angle2

def right_angle():
    thresh = 0.005
    rospy.sleep(1)
    while True:
        if angle_120 - angle_60 >thresh:
            turn_right(0.5, 0.1)
        elif angle_120 - angle_60 <-thresh:
            turn_left(0.5, 0.1)
        # elif diff(angle_60*m, angle_90) >1:
        #     turn_left(0.1, 0.1)
        else:
            print(angle_60, angle_120, angle_120-angle_60)
            print("done?")
            break

if __name__=="__main__":
    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)
    right_angle()
    # rospy.spin()