#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

x = 1
y = 1
z = 1
init_pose = [0, 0]
i = 0

def get_current_position(msg):
    global x, y, z, init_pose, i

    pose = msg.pose.pose.position
    
    if i < 5:
        init_pose = [pose.x, pose.y]
        print(f'init pose: {init_pose}')

    x = pose.x
    y = pose.y
    z = pose.z
    i = i+1

    return x, y, z


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

def get_goal(init_pose, x_var, y_var, distance):
    var = (distance**2 / (x_var**2+y_var**2))**0.5
    goal = [init_pose[0] + var*x_var,init_pose[1] + var*y_var]
    return goal

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_fixed_pose')
        odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, get_current_position)
        rospy.sleep(1)
        goal = get_goal(init_pose, -0.293, -0.880, 1)
        print(goal)
        result = go_to_goal(goal[0], goal[1])
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
