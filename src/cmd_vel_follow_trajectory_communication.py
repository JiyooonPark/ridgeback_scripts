#!/usr/bin/env python3
from tools import tools_cmd_vel
import math
import rospy
import time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

x = 0
y = 0
w = 0
PI = math.pi


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

    i = 0
    send_message['status'] = 'executing'
    ir_pub.publish(f'{send_message["number"]} {send_message["status"]} ')
    while (True):

        i += 1

        K_linear = 0.2
        x_dist = x_goal - x
        y_dist = y_goal - y
        distance = math.hypot(x_dist, y_dist)

        velocity_message.linear.x = K_linear * (x_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.linear.y = K_linear * (y_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.angular.z = 0
        velocity_publisher.publish(velocity_message)

        # if i % 15000 == 0:
            # print(f'CUR: x: {round(x, 2)} y: {round(y, 2)} distance: {round(distance, 2)} x_dist: {round(x_dist, 2)} y_dist: {round(y_dist, 2)}')

        if (distance < 0.4):
            print('done')
            return True

    return False


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
            ir_pub.publish(f'{send_message["number"]} {send_message["status"]}')
            rospy.loginfo("Goal execution done!")

            # while received_message['executed'] != 'done':
            send_message['status'] = 'waiting'
            ir_pub.publish(f'{send_message["number"]} {send_message["status"]}')
                # rospy.loginfo(f'iiwa is done {recieved_message["executed"]} now starting {send_message["executed"]+1}')
        else:
            pass
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        rospy.init_node('drawing_ridgeback')

        odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, get_current_position)

        ir_sub = rospy.Subscriber('/iiwa_ridgeback_communicaiton/iiwa', String, get_communication)
        ir_pub = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback', String, queue_size=10)
        # tools_cmd_vel.turn_left(0.1, PI *5)

        rospy.sleep(0.5)
        x_points = [-1.649, -1.532, -1.449, -1.395, -1.361, -1.341, -1.328, -1.316, -1.296, -1.262, -1.207, -1.125, -1.008, -0.862, -0.696, -0.513, -0.313, -0.1, 0.127, 0.364, 0.612, 0.868, 1.13, 1.398, 1.669, 1.923, 2.142, 2.333, 2.501, 2.652, 2.79, 2.923, 3.054, 3.19, 3.337, 3.499, 3.683, 3.857, 3.991, 4.096, 4.18, 4.253, 4.322, 4.398, 4.489, 4.605, 4.754, 4.946, 5.189, 5.423, 5.591, 5.706, 5.781, 5.829, 5.864, 5.9, 5.949, 6.024, 6.14, 6.308, 6.544, 6.79, 6.987, 7.146, 7.276, 7.387, 7.489, 7.59, 7.701, 7.831, 7.99, 8.187, 8.433, 8.676, 8.864, 9.009, 9.121, 9.211, 9.291, 9.371, 9.461, 9.574, 9.718, 9.907]
        y_points = [5.859, 5.929, 5.978, 6.011, 6.03, 6.042, 6.049, 6.056, 6.067, 6.087, 6.119, 6.169, 6.239, 6.3, 6.327, 6.324, 6.401, 6.362, 6.316, 6.27, 6.23, 6.204, 6.195, 6.216, 6.272, 6.342, 6.399, 6.444, 6.452, 6.469, 6.471, 6.457, 6.427, 6.38, 6.314, 6.23, 6.126, 6.011, 5.914, 5.829, 5.756, 5.694, 5.643, 5.602, 5.572, 5.553, 5.544, 5.545, 5.583, 5.619, 5.691, 5.791, 5.913, 6.048, 6.19, 6.33, 6.461, 6.577, 6.668, 6.736, 6.758, 6.748, 6.718, 6.674, 6.619, 6.557, 6.49, 6.427, 6.364, 6.309, 6.265, 6.236, 6.225, 6.24, 6.281, 6.339, 6.417, 6.504, 6.597, 6.69, 6.778, 6.855, 6.917, 6.957]
        x_positive = [x for x in x_points]
        y_positive = [abs(y) - 2.2 for y in y_points]

        # x_positive = [0, 0, 0]
        # y_positive = [0, 1, 3]

        path = zip(x_positive, y_positive)

        with_iiwa_follow_traj(path, ir_pub)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
