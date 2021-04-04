#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math

roll = pitch = yaw = 0.0
target = 60
kp=0.5

def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    # print orientation_q
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    angle = yaw *180 / math.pi
    print angle

rospy.init_node('rotate_robot')

sub = rospy.Subscriber ('amcl_pose', PoseWithCovarianceStamped, get_rotation)
r = rospy.Rate(10)
command =Twist()
rospy.spin()
