#!/usr/bin/env python3

import rospy
from tools import tools_cmd_vel
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math

i = 0
angle = 0
rad = 0
pose_x = 0
pose_y = 0


def callback_odom(msg):
    global i, angle, rad
    global pose_x, pose_y
    pose_x = msg.pose.pose.position.x
    pose_y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    if yaw<0:
        angle = -yaw*180/math.pi
    else:
        angle = 360-yaw*180/math.pi
    rad = math.radians(angle)

    
if __name__ == "__main__":
    rospy.init_node('right_place')

    odom_sub = rospy.Subscriber ('/odom', Odometry, callback_odom)
    rospy.sleep(3)
    
    try:
        x_goal = 0
        y_goal = 0

        x_move = x_goal-pose_x
        y_move = y_goal-pose_y
        print(f'GOAL: {x_goal},{y_goal} CURRENT: {round(pose_x,2)}{round(pose_y,2)}')
        
        goal = np.array([[x_move],[y_move]])
        rospy.sleep(1)
        rotation_matrix = np.array([[math.cos(rad), -math.sin(rad)], 
                [math.sin(rad), math.cos(rad)]])
        print (rotation_matrix)
        r_goal = np.matmul(rotation_matrix, goal)
        print('new goal', r_goal)
        tools_cmd_vel.move_relative(float(r_goal[0][0]), float(r_goal[1][0]), duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")




#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

x = 1
y = 1
w = 1


def get_current_position(msg):
    global x, y, w
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w = msg.pose.pose.orientation.w
    return x, y, w

def go_to_goal(x_goal, y_goal):

    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)

    i = 0
    while (True):
        i += 1

        K_linear = 0.05
        x_dist = x_goal - x
        y_dist = y_goal - y
        distance = math.hypot(x_dist, y_dist)

        velocity_message.linear.x = K_linear * (x_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.linear.y = K_linear * (y_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.angular.z = 0

        velocity_publisher.publish(velocity_message)
        if i% 10000 == 0:
            print(f'CUR: x: {round(x, 2)} y: {round(y, 2)} distance: {round(distance)}')

        if (distance < 0.05):
            print('done')
            return True
    return False

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_fixed_pose')
        odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, get_current_position)
        rospy.sleep(0.5)
        result = go_to_goal(2, 2)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
