#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

global x, y, w
x = 1
y = 1
w = 1

def get_current_position(msg):
    global x, y, w
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w = msg.pose.pose.orientation.w

def go_to_goal(x_goal, y_goal):

    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)
    i = 0
    while (True):

        i = i + 1

        K_linear = 0.05
        x_dist = x_goal - x
        y_dist = y_goal - y
        distance = math.hypot(x_dist, y_dist)

        velocity_message.linear.x = K_linear * (x_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.linear.y = K_linear * (y_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.angular.z = 0

        velocity_publisher.publish(velocity_message)

        if i % 100000 == 0:
            print(
                f'CUR: x: {round(x, 2)} y: {round(y, 2)} distance: {round(distance, 2)} x_dist: {round(x_dist, 2)} y_dist: {round(y_dist, 2)}')

        if (distance < 0.01):
            print('done')
            break

def follow_traj(path):
    for (px, py) in path:
        result = go_to_goal(px, py)
        if result:
            rospy.loginfo("Goal execution done!")
        else:
            pass
        rospy.sleep(0.5)
        print(x, y, w)

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_fixed_pose')
        odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, get_current_position)
        rospy.sleep(0.5)

        x_positive = [7, 8] #[0, 1, 4, 6]
        y_positive = [1, 0] # [0, -2, 2, 4]

        path = zip(x_positive, y_positive)

        follow_traj(path)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
