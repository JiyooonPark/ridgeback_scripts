#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time, math

from tools import *
from tool_callbacks import *

PI = math.pi

def publish_cmd_vel():

    goal = 0.6
    speed = 1

    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

    cmd = Twist()
    # always set to 0.05 for optimal result
    cmd.linear.x = - speed
    cmd.linear.y = -speed
    cmd.linear.z = 0

    rospy.sleep(1)

    seconds = time.time()
    while time.time() - seconds <goal/speed:
        publisher.publish(cmd)


def callback_laser(msg):

    # 1. check the range of laser scanner and change the code accordingly
    max_range = len(msg.ranges)
    # print (len(msg.ranges))
    # gazebo simulator : 0-720 


    # 2. to check distance
    # values at 0 degree
    # print ("0:{:.4}".format(msg.ranges[0]))
    # # values at 90 degree
    # print ("90:{:.4}".format(msg.ranges[max_range/2]))
    # # values at 180 degree
    # print ("180:{:.4}".format(msg.ranges[max_range-1]))


    # 3. check 60, 90, 120 distance
    angle_60 = average(msg.ranges, round(max_range/3))
    angle_90 = average(msg.ranges, round(max_range/2))
    angle_120 = average(msg.ranges, round(max_range*2/3))

    print("{:.4} {:.4} {:.4}".format(angle_60, angle_90, angle_120))

def turn_right(degrees):
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    speed = 0.5
    seconds = degrees/speed

    cmd = Twist()

    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = -speed  # in radians 

    seconds = time.time()
    while time.time() - seconds < degrees / speed:
        publisher.publish(cmd)


if __name__=="__main__":
    
    rospy.init_node('holonimoic_move_to_goal')
    odom_sub = rospy.Subscriber('/odom', Odometry, callback_odom)
    laser_sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)
    degrees_20 = PI/9
    turn_right(degrees_20)
    rospy.spin()
