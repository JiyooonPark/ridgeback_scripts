#! /usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy
from tools import tools_cmd_vel

# ========= GLOBAL VARIABLES =========== #
angle_90 = 0
angle_60 = 0
angle_120 = 0

def callback_laser(msg):
    global angle_120, angle_60, angle_90

    # for real ridgeback
    # angle_60 = (msg.ranges[490]+msg.ranges[500]+msg.ranges[510])/3
    # angle_90 = (msg.ranges[530]+msg.ranges[540]+msg.ranges[550])/3
    # angle_120 = (msg.ranges[570]+msg.ranges[580]+msg.ranges[590])/3

    # for simulation
    angle_60 = (msg.ranges[230]+msg.ranges[240]+msg.ranges[250])/3
    angle_90 = (msg.ranges[350]+msg.ranges[360]+msg.ranges[370])/3
    angle_120 = (msg.ranges[470]+msg.ranges[480]+msg.ranges[490])/3

def right_angle():
    thresh = 0.008
    rospy.sleep(1)
    while True:
        print(angle_60, angle_90, angle_120)
        if angle_120 - angle_60 > thresh:
            tools_cmd_vel.turn_right(0.05, 0.1)
        elif angle_120 - angle_60 < -thresh:
            tools_cmd_vel.turn_left(0.05, 0.1)
        else:
            print(angle_60, angle_120, angle_120-angle_60)
            print("done?")
            break


if __name__ == "__main__":
    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)
    right_angle()
    # rospy.spin()
