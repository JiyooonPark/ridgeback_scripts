#! /usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy

from tools import tools_cmd_vel
from tools import tools_etc

angle_70 = 0
angle_90 = 0
angle_110 = 0
m = 0


# laser scanner


def callback_laser(msg):
    global angle_110, angle_70, angle_90, m

    angle_70 = tools_etc.average(msg.ranges, 70)
    angle_90 = tools_etc.average(msg.ranges, 90)
    angle_110 = tools_etc.average(msg.ranges, 110)

    # print("70 : {:.4} 90: {:.4} 110: {:.4}".format(angle_70, angle_90, angle_110))


def keep_align():
    sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)

    speed = 0.05
    duration = 0.01

    thresh = 0.05
    rospy.sleep(1)

    while True:
        if angle_110 - angle_70 > thresh:
            tools_cmd_vel.turn_right(speed, duration)
        elif angle_110 - angle_70 < -thresh:
            tools_cmd_vel.turn_left(speed, duration)
        else:
            print(angle_70, angle_110, angle_110 - angle_70)
            print("done")
            break


if __name__ == "__main__":
    rospy.init_node('cylinder_laser_right')

    keep_align()
    # rospy.spin()
