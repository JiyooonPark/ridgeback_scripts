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
        velocity_publisher.publish(velocity_message)

        # if i % 15000 == 0:
            # print(f'CUR: x: {round(x, 2)} y: {round(y, 2)} distance: {round(distance, 2)} x_dist: {round(x_dist, 2)} y_dist: {round(y_dist, 2)}')

        if (distance < 0.4):
            print('done')
            return True

    return False


def with_iiwa_follow_traj(path, angle_list, ir_pub):
    global send_message
    send_message['number'] = 0
    angle_index = 0

    for (px, py) in path:
        print(px, py)
        print(angle_index , ' ', angle_list[angle_index])
        if px > 90 and py < -90:
            if angle_list[angle_index] > 0:
                # tools_cmd_vel.turn_right(90)
                tools_cmd_vel.turn_right(angle_list[angle_index])
            elif angle_list[angle_index] == 0:
                # tools_cmd_vel.turn_left(90)
                pass
            else:
                # tools_cmd_vel.turn_left(90)
                tools_cmd_vel.turn_left(-angle_list[angle_index])
            print('drawing')
            time.sleep(16)
            
            continue
        elif px <- 90 and py > 90:
            if angle_list[angle_index] > 0:
                # tools_cmd_vel.turn_left(90)
                tools_cmd_vel.turn_left(angle_list[angle_index])
            elif angle_list[angle_index] == 0:
                # tools_cmd_vel.turn_right(90)
                pass
            else:
                # tools_cmd_vel.turn_right(90)
                tools_cmd_vel.turn_right(-angle_list[angle_index])
            angle_index+=1
            continue
        
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
        # rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        rospy.init_node('drawing_ridgeback')

        odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, get_current_position)

        ir_sub = rospy.Subscriber('/iiwa_ridgeback_communicaiton/iiwa', String, get_communication)
        ir_pub = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback', String, queue_size=10)
        # tools_cmd_vel.turn_left(0.1, PI *5)

        rospy.sleep(0.5)
        # x_points = [-0.513, -0.313, -0.1, 0.127, 0.364, 100, -100, 0.612, 0.868, 1.13, 1.398, 1.669, 100, -100, 1.923, 2.142, 2.333, 2.501, 2.652, 2.79, 2.923, 3.054, 100, -100, 3.19, 3.337, 3.499, 100, -100, 3.683, 3.857, 3.991, 4.096, 4.18, 4.253, 4.322, 4.398, 4.489, 4.605, 100, -100, 5.000, 100, -100, 5.56, 100, -100, 6.3,  6.79, 6.987, 7.146, 7.276, 7.387, 7.489, 7.59, 7.701, 100, -100, 7.831, 7.99, 8.187, 8.433, 8.676, 100, -100, 8.864, 9.009, 9.121, 9.211, 9.291, 9.371, 9.461, 9.574, 9.718, 100, -100, 9.907, 100]
        # y_points = [6.556, 6.533, 6.494, 6.448, 6.402, 102.2, -97.8, 6.369, 6.343, 6.337, 6.358, 6.414, 102.2, -97.8, 6.469, 6.526, 6.571, 6.603, 6.62, 6.622, 6.608, 6.578, 102.2, -97.8, 6.467, 6.401, 6.317, 102.2, -97.8, 6.229, 6.121, 6.024, 5.939, 5.866, 5.804, 5.753, 5.712, 5.682, 5.663, 102.2, -97.8, 5.900, 102.2, -97.8, 5.900, 102.2, -97.8, 5.9, 6.85, 6.82, 6.776, 6.721, 6.659, 6.592, 6.526, 6.463, 102.2, -97.8, 6.431, 6.387, 6.358, 6.347, 6.362, 102.2, -97.8, 6.614, 6.675, 6.753, 6.84, 6.933, 7.026, 7.114, 7.191, 7.253, 102.2, -97.8, 7.273, 102.2]
        # angle_list = [-30, 0, 0, 0, 45, 0, 0, -30, -45, -30, 45, -30]


        # x_points = [-0.513, -0.313, -0.1, 0.127, 0.364, 100, -100, 0.612, 0.868, 1.13, 1.398, 1.669, 100, -100, 1.923, 2.142, 2.333, 2.501, 2.652, 2.79, 2.923, 3.054, 100, -100, 3.19, 3.337, 3.499, 100, -100, 3.683, 3.857, 3.991, 4.096, 4.18, 4.253, 4.322, 4.398, 4.489, 4.605, 100, -100, 4.754, 4.946, 5.189, 5.423, 5.591, 5.706, 5.781, 5.829, 100, -100, 5.864, 5.9, 5.949, 6.024, 6.14, 6.308, 6.544, 100, -100, 6.79, 6.987, 7.146, 7.276, 7.387, 7.489, 7.59, 7.701, 100, -100, 7.831, 7.99, 8.187, 8.433, 8.676, 100, -100, 8.864, 9.009, 9.121, 9.211, 9.291, 9.371, 9.461, 9.574, 9.718, 100, -100, 9.907, 100]
        # y_points = [4.28, 4.257, 4.218, 4.172, 4.126, 102.2, -97.8, 4.079, 4.053, 4.047, 4.068, 4.124, 102.2, -97.8, 4.209, 4.266, 4.311, 4.343, 4.36, 4.362, 4.348, 4.318, 102.2, -97.8, 4.335, 4.269, 4.185, 102.2, -97.8, 4.065, 3.957, 3.86, 3.775, 3.702, 3.64, 3.589, 3.548, 3.518, 3.499, 102.2, -97.8, 3.195, 3.196, 3.207, 3.243, 3.315, 3.415, 3.537, 3.672, 102.2, -97.8, 3.745, 3.885, 4.016, 4.132, 4.223, 4.283, 4.305, 102.2, -97.8, 4.632, 4.602, 4.558, 4.503, 4.441, 4.374, 4.308, 4.245, 102.2, -97.8, 4.167, 4.123, 4.094, 4.083, 4.098, 102.2, -97.8, 3.928, 3.989, 4.067, 4.154, 4.247, 4.34, 4.428, 4.505, 4.567, 102.2, -97.8, 4.627, 102.2]
        
        # -0.8640000000000001, 0.246, 1.624, 2.834, 3.329, 4.322, 5.604, 6.465, 7.417, 8.433, 9.663
        x_points = [-0.864, 0.246, 1.624, 2.834, 3.329, 4.322, 5.604, 6, 6.465, 7.417, 8.433, 9.663]
        y_points = [5.008, 4.909, 4.932, 5.109, 4.803, 4.293, 4.325, 4.3, 5.331, 5.089, 4.837, 5.493]
        
        # x_points = [ -0.513, -0.313, -0.1, 0.127, 0.364, 100, -100, 0.612, 0.868, 1.13, 1.398, 1.669, 100, -100, 1.923, 2.142, 2.333, 2.501, 2.652, 2.79, 2.923, 3.054, 100, -100, 3.19, 3.337, 3.499, 100, -100, 3.683, 3.857, 3.991, 4.096, 4.18, 4.253, 4.322, 4.398, 4.489, 4.605, 100, -100, 4.754, 4.946, 5.189, 5.423, 5.591, 5.706, 5.781, 5.829, 5.864, 5.9, 5.949, 100, -100, 6.024, 6.14, 6.308, 6.544, 100, -100, 6.79, 6.987, 7.146, 7.276, 7.387, 7.489, 7.59, 7.701, 100, -100, 7.831, 7.99, 8.187, 8.433, 8.676, 100, -100, 8.864, 9.009, 9.121, 9.211, 9.291, 9.371, 9.461, 9.574, 9.718, 9.907, 100]
        # y_points = [4.255, 4.232, 4.193, 4.147, 4.101, 100.2, -99.8, 4.063, 4.037, 4.031, 4.052, 4.108, 100.2, -99.8, 4.19, 4.247, 4.292, 4.324, 4.341, 4.343, 4.329, 4.299, 100.2, -99.8, 4.283, 4.217, 4.133, 100.2, -99.8, 4.036, 3.928, 3.831, 3.746, 3.673, 3.611, 3.56, 3.519, 3.489, 3.47, 100.2, -99.8, 3.006, 3.007, 3.018, 3.054, 3.126, 3.226, 3.348, 3.483, 3.625, 3.765, 3.896, 100.2, -99.8, 3.947, 4.038, 4.098, 4.12, 100.2, -99.8, 4.631, 4.601, 4.557, 4.502, 4.44, 4.373, 4.307, 4.244, 100.2, -99.8, 4.177, 4.133, 4.104, 4.093, 4.108, 100.2, -99.8, 4.018, 4.079, 4.157, 4.244, 4.337, 4.43, 4.518, 4.595, 4.657, 4.697, 100.2]
        
        angle_list=[-30, 0, 0, 0, 45, 0, 0, -30, 45, 0, -30]
        
        x_points =[-0.553, 100, -100, 0.246, 100, -100, 1.624, 100, -100, 2.834, 100, -100, 2.79, 100, -100, 4.322, 100, -100, 5.604, 100, -100,6, 6.776, 100, -100, 6.878, 100, -100, 8.433, 100, -100, 9.974, 100, -100]
        y_points=[4.469, 100.2, -99.8, 4.287, 100.2, -99.8, 4.31, 100.2, -99.8, 4.487, 100.2, -99.8, 4.492, 100.2, -99.8, 3.671, 100.2, -99.8, 3.703, 100.2, -99.8,3.7, 4.792, 100.2, -99.8, 4.778, 100.2, -99.8, 4.215, 100.2, -99.8, 4.954, 100.2, -100]
        x_positive = [x-0.1 for x in y_points]
        y_positive = [-(y-0.1) for y in x_points]


        path = zip(x_positive, y_positive)
        # tools_cmd_vel.turn_left(90)

        with_iiwa_follow_traj(path, angle_list, ir_pub)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
