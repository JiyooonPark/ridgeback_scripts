#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math

global x, y, w
x = 0
y = 0
w = 0


def get_current_position(msg):
    global x, y, w
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w = msg.pose.pose.orientation.w
    print('in function', x, y, w)
    return x, y, w


def go_to_goal(x_goal, y_goal):
    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)
    print('current position: ', x, y)

    i = 0
    while (True):
        i = i + 1

        K_linear = 0.01
        distance = abs(math.sqrt(((x_goal - x) ** 2) + ((y_goal - y) ** 2)))

        x_dist = x_goal - x
        y_dist = y_goal - y

        velocity_message.linear.x = K_linear * \
                                    (x_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.linear.y = K_linear * \
                                    (y_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.angular.z = 0

        velocity_publisher.publish(velocity_message)
        # print ('x=', x, 'y=',y)
        if i % 300000 == 0:
            print('distance', distance)
            print('current location', x, y)

        else:
            continue

        if (distance < 0.02):
            print('done')
            break


if __name__ == '__main__':
    try:
        rospy.init_node('move_to_fixed_pose')
        amcl_sub = rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped, get_current_position)
        rospy.sleep(0.5)
        # print(x, y,w)
        result = go_to_goal(0, 0)
        if result:
            rospy.loginfo("Goal execution done!")
            rospy.sleep(0.5)
            print(x, y, w)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
