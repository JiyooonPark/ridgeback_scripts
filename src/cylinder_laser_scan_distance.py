#! /usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy

from tools import *
from tools_cmd_vel import *

angle_90=0

# laser scanner 
def callback_laser(msg):
    global angle_90

    angle_90 = average(msg.ranges, 90)
    # print(angle_90)


def keep_distance(distance):

    speed = 0.05
    duration = 0.05

    thresh = 0.05
    rospy.sleep(1)

    while True:
        if angle_90 - distance > thresh: # 1.3m - 1m  = 0.3 m > 0.1
            print(angle_90, end=": ")
            move_forward(speed, duration)
        elif angle_90 - distance < -thresh: # 0.7m - 1m  = -0.3 m < -0.1
            print(angle_90, end=": ")
            move_backward(speed, duration)
        else:
            print("distance to cylinder :", angle_90)
            print("done")
            break

if __name__=="__main__":

    rospy.init_node('cylinder_laser_right')

    sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)
    keep_distance(1.0)
    # rospy.spin()