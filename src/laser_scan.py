#! /usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy, math


def callback_laser(msg):
    # m = math.sqrt(3)/2
    # # print (len(msg.ranges))
    # # values at 0 degree
    # print ("0:{:.4}".format(msg.ranges[240]))
    # # values at 90 degree
    # print ("90:{:.4}".format(msg.ranges[360]))
    # # values at 180 degree
    # print ("180:{:.4}".format(msg.ranges[480]))
    # print()
    # m = math.sqrt(3)/2
    angle_60 = (msg.ranges[490]+msg.ranges[500]+msg.ranges[510])/3
    angle_90 = (msg.ranges[530]+msg.ranges[540]+msg.ranges[550])/3
    angle_120 = (msg.ranges[570]+msg.ranges[580]+msg.ranges[590])/3

    print("{:.4} {:.4} {:.4}".format(angle_120, angle_90, angle_60))
def right_angle():
    pass

if __name__=="__main__":
    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)
    rospy.spin()