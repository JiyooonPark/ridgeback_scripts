#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

roll = pitch = yaw = 0.0
angle=0
i=0

def get_rotation (msg):

    global roll, pitch, yaw, angle

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    if yaw<0:
        angle = -yaw*180/math.pi

    else:
        angle = 360-yaw*180/math.pi

    if i%30==0:
        print(f"angle : {round(angle, 2)}")

if __name__=='__main__':
        
    rospy.init_node('rotate_robot')

    sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
    r = rospy.Rate(10)
    rospy.spin()

    