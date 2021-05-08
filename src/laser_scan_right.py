#! /usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy
import math, time
from geometry_msgs.msg import Twist

angle_90=0
angle_60=0
angle_120=0
m = 0

def callback_laser(msg):
    global angle_120, angle_60, angle_90, m
    # print ("0:{:.4}".format(msg.ranges[0]))
    # print ("90:{:.4}".format(msg.ranges[360]))
    # print ("180:{:.4}".format(msg.ranges[719]))
    # print()
    angle_60 = msg.ranges[240]
    angle_90 = msg.ranges[360]
    angle_120 = msg.ranges[480]
    m = math.sqrt(3)/2

    # print("{:.4} {:.4} {:.4}".format(angle_120, angle_90, angle_60))

def turn_right(z, sec):
    print("right")
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    cmd = Twist()
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = -z
    rospy.sleep(1)
    # z_init=z
    # print("init z: {:.3f}".format(  to_angle(z_init)))
    seconds = time.time()
    while time.time() - seconds <sec:
        publisher.publish(cmd)
def turn_left(z, sec):
    print("left")
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    cmd = Twist()
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = z
    rospy.sleep(1)
    # z_init=z
    # print("init z: {:.3f}".format(  to_angle(z_init)))
    seconds = time.time()
    while time.time() - seconds <sec:
        publisher.publish(cmd)
def diff(angle1, angle2):
    print("difference of : {:.4} and {:.4} is {:.4}".format(angle1, angle2, angle1-angle2))
    return angle1-angle2
def right_angle():
    rospy.sleep(1)
    
    while True:
        if diff(angle_120*m, angle_90) >0.01:
            turn_right(0.05, 0.1)
        elif diff(angle_120*m, angle_90) <-0.01:
            turn_left(0.05, 0.1)
        # elif diff(angle_60*m, angle_90) >1:
        #     turn_left(0.1, 0.1)
        else:
            print("done?")
            break

if __name__=="__main__":
    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)
    right_angle()
    rospy.spin()