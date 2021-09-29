#!/usr/bin/env python3

from scipy import interpolate
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import math, time
from nav_msgs.msg import Odometry
from std_msgs.msg import String

global x, y, w
x = 0
y = 0
w = 0

received_message = {}
send_message = {}

'''
number: instruction number
status: executing, done, error, waiting
==============
4 done
'''


def get_current_position(msg):
    global x, y, w
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w = msg.pose.pose.orientation.w
    return x, y, w

def get_communication(msg):
    global received_message
    msg_data = msg.split()
    received_message['number'] = int(msg_data[0])
    received_message['status'] = msg_data[1]

def go_to_goal(x_goal, y_goal, ir_pub):
    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)
    print('current position: ', x, y)

    i = 0
    send_message['status'] = 'executing'
    ir_pub.publish(f'{send_message["number"]} {send_message["status"]} ')
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
        print(f'CUR: x: {x} y: {y} distance: {distance} x_dist: {x_dist} y_dist: {y_dist}')

        if (distance < 0.1):
            print('done')
            return True
    return False

def follow_traj(path):

    for (px, py) in path:
        result = go_to_goal(px, py)
        if result:
            rospy.loginfo("Goal execution done!")
        else:
            pass
        rospy.sleep(0.5)
        print(x, y, w)

def with_iiwa_follow_traj(path, ir_pub):
    global send_message
    send_message['number'] = 0
    for (px, py) in path:
        send_message['number'] += 1
        send_message['status'] = 'started'
        ir_pub.publish(f'{send_message["number"]} {send_message["status"]}')
        result = go_to_goal(px, py, ir_pub)
        if result:
            send_message['status'] = 'done'
            rospy.loginfo("Goal execution done!")
            ir_pub.publish(f'{send_message["number"]} {send_message["status"]}')
            # while received_message['executed'] != 'done':
            send_message['status']='waiting'
            ir_pub.publish(f'{send_message["number"]} {send_message["status"]}')
            # rospy.loginfo(f'iiwa is done {recieved_message["executed"]} now starting {send_message["executed"]+1}')
        else:
            pass
        rospy.sleep(0.5)
        print(x, y, w)


def f(xx):
    x_points = [-1.65, -1.25, -0.88, -0.34, 0.13, 0.64, 1.19, 1.78, 2.0, 2.58, 3.05, 3.39, 3.96, 4.33, 4.74, 5.24, 5.86,
                6.09, 6.36, 6.76, 7.29, 7.72, 8.21, 8.88, 9.02, 9.49, 9.96]
    y_points = [-4.75, -4.99, -5.19, -5.2, -5.11, -5.02, -5.0, -5.1, -5.16, -5.28, -5.25, -5.11, -4.76, -4.47, -4.37,
                -4.39, -4.76, -5.38, -5.53, -5.54, -5.41, -5.15, -5.02, -5.08, -5.14, -5.59, -5.75]

    y_positive = [abs(y) for y in y_points]

    tck = interpolate.splrep(x_points, y_positive)
    return interpolate.splev(xx, tck)


if __name__ == '__main__':
    try:
        rospy.init_node('move_to_fixed_pose')
        odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, get_current_position)

        ir_sub = rospy.Subscriber('/iiwa_ridgeback_communicaiton/iiwa', String, get_communication)
        ir_pub = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback', String, queue_size=10)
        rospy.sleep(0.5)

        # x_points = [-1.65, -1.25, -0.88, -0.34, 0.13]
            # , 0.64, 1.19, 1.78, 2.0, 2.58, 3.05, 3.39, 3.96, 4.33, 4.74, 5.24, 5.86, 6.09, 6.36, 6.76, 7.29, 7.72, 8.21, 8.88, 9.02, 9.49, 9.96]
        # y_points = [-4.75, -4.99, -5.19, -5.2, -5.11]
            # , -5.02, -5.0, -5.1, -5.16, -5.28, -5.25, -5.11, -4.76, -4.47,
            #         -4.37, -4.39, -4.76, -5.38, -5.53, -5.54, -5.41, -5.15, -5.02, -5.08, -5.14, -5.59, -5.75]

        # x_positive = [x for x in x_points]
        # y_positive = [abs(y) - 0.8 for y in y_points]
        x_positive = [0, 1, 2]
        y_positive = [0, 1, 2]

        path = zip(x_positive, y_positive)

        with_iiwa_follow_traj(path, ir_pub)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
