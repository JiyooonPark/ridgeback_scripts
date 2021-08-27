#!/usr/bin/env python3

from scipy import interpolate
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
    # print('in function',  x, y, w)
    return x, y, w


def go_to_goal(x_goal, y_goal):

    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)
    print('current position: ', x, y)

    i = 0
    while (True):
        i = i+1

        K_linear = 0.1
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        x_dist = x_goal - x
        y_dist = y_goal - y

        velocity_message.linear.x = K_linear * \
            (x_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.linear.y = K_linear * \
            (y_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.angular.z = 0

        velocity_publisher.publish(velocity_message)
        # print ('x=', x, 'y=',y)
        # if i % 700000 == 0:
            # print('distance', distance)
            # print('current location', x, y)

        # else:
            # continue

        if (distance < 0.3):
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


def f(xx):
    # x_points = [9.96, 9.94, 9.51, 8.68, 8.24, 7.98, 6.48, 6.06, 5.02, 4.6, 4.27, 3.36, 2.62, 1.71, 0.46, 0.38, -0.07, -0.94, -1.21, -2.67, -3.24, -3.74]
    # y_points = [-7.28, -7.19, -5.77, -5.17, -3.75, -3.0, -3.33, -4.39, -5.42, -6.45, -5.19, -3.97, -4.27, -4.59, -5.8, -5.05, -6.34, -5.69, -5.47, -5.37, -5.97, -4.73]

    x_points = [-3.08, -1.39, -0.18, 0.08, 1.73, 1.84, 3.78, 4.51, 4.69, 6.04, 6.84, 8.16, 8.78, 9.24, 10.05]
    y_points = [-5.66, -4.98, -6.02, -5.44, -4.3, -4.32, -4.03, -6.07, -5.22, -4.1, -2.57, -3.06, -5.01, -5.22, -6.93]

    x_points = [-2.94, -0.05, 2.09, 4.72, 6.04, 6.68, 8.91, 10.05]
    y_points = [-5.11, -5.44, -3.89, -5.52, -3.4, -2.37, -4.54, -5.93]

    y_positive = [abs(y) for y in y_points]

    tck = interpolate.splrep(x_points, y_positive)
    return interpolate.splev(xx, tck)


if __name__ == '__main__':
    try:
        rospy.init_node('move_to_fixed_pose')
        amcl_sub = rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped, get_current_position)
        rospy.sleep(0.5)
        # path_x = [0.4 * n for n in range(-5, 12)]
        # path_y = [float(f(n)) for n in path_x]
        x_points = [-1.65, -1.25, -0.88, -0.34, 0.13, 0.64, 1.19, 1.78, 2.0, 2.58, 3.05, 3.39, 3.96, 4.33, 4.74, 5.24, 5.86, 6.09, 6.36, 6.76, 7.29, 7.72, 8.21, 8.88, 9.02, 9.49, 9.96]
        y_points = [-4.75, -4.99, -5.19, -5.2, -5.11, -5.02, -5.0, -5.1, -5.16, -5.28, -5.25, -5.11, -4.76, -4.47, -4.37, -4.39, -4.76, -5.38, -5.53, -5.54, -5.41, -5.15, -5.02, -5.08, -5.14, -5.59, -5.75]
        x_positive = [x+0.25 for x in x_points]
        y_positive = [abs(y)-0.7 for y in y_points]
        path = zip(x_positive, y_positive)
        follow_traj(path)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
