#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import math

# ========= GLOBAL VARIABLES =========== #
roll = pitch = yaw = 0.0

target = 0
angle=-1010
kp=0.5

def get_rotation (msg):

    global roll, pitch, yaw, angle

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    if yaw<0:
        angle = -yaw*180/math.pi

    else:
        angle = 360-yaw*180/math.pi

if __name__=='__main__':

    rospy.init_node('fixed_orientation')

    sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    r = rospy.Rate(10)
    command =Twist()

    if 0<=target<=180:
        real_target = -target
    else:
        real_target = 360-target

    # print(f'target: {target}, real_target: {real_target}')
    
    while True:

        target_rad = real_target*math.pi/180

        command.angular.z = kp * (target_rad-yaw)
        
        pub.publish(command)

        r.sleep()

        if abs(yaw*180/math.pi-real_target)<2:
            break
        
    print("done")