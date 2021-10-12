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
    xx = y_goal
    yy = -x_goal
    while (True):

        i += 1

        K_linear = 0.2
        x_dist = xx - x
        y_dist = y - yy
        
        distance = math.hypot(x_goal - x, y_goal-y)

        velocity_message.linear.x = K_linear * (x_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.linear.y = K_linear * (y_dist / (abs(x_dist) + abs(y_dist)))
        velocity_publisher.publish(velocity_message)

        if i % 400000 == 0:
            print(f'x dist: {x_goal - x}, y dist: {y_goal-y}')
            print(distance)
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

        if (px, py) == (-100, -100):
            if angle_list[angle_index] < 0:
                tools_cmd_vel.turn_right(-angle_list[angle_index])
            else:
                tools_cmd_vel.turn_left(angle_list[angle_index])

            angle_index+=1
            # tools_cmd_vel.move_relative(0,-1.2)
            continue
        elif (px, py) == (100, 100):
            # tools_cmd_vel.move_relative(0,1.2)
            if angle_list[angle_index] < 0:
                tools_cmd_vel.turn_left(-angle_list[angle_index])
            else:
                tools_cmd_vel.turn_right(angle_list[angle_index])
            tools_cmd_vel.turn_left(angle_list[angle_index])
            print('drawing')
            # time.sleep(2)
            continue
        
        send_message['number'] += 1
        send_message['status'] = 'started'
        ir_pub.publish(f'{send_message["number"]} {send_message["status"]}')

        result = go_to_goal(px,py, ir_pub)

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
        x_points = [-1.008, 100, -100, -0.862, -0.696, -0.513, -0.313, -0.1, 0.127, 100, -100, 0.364, 0.612, 0.868, 1.13, 1.398, 100, -100, 1.669, 1.923, 2.142, 2.333, 2.501, 2.652, 100, -100, 2.79, 2.923, 3.054, 3.19, 100, -100, 3.337, 3.499, 3.683, 3.857, 3.991, 4.096, 4.18, 4.253, 4.322, 100, -100, 4.398, 4.489, 4.605, 4.754, 4.946, 5.189, 100, -100, 5.423, 5.591, 5.706, 5.781, 5.829, 5.864, 5.9, 5.949, 6.024, 6.14, 100, -100, 6.308, 100, -100, 6.544, 6.79, 6.987, 7.146, 7.276, 100, -100, 7.387, 7.489, 7.59, 7.701, 7.831, 7.99, 8.187, 100, -100, 8.433, 8.676, 8.864, 9.009, 9.121, 9.211, 9.291, 9.371, 100, -100, 9.461, 9.574, 9.718, 9.907, 100]
        y_points = [6.602, 103, -97, 6.54, 6.567, 6.564, 6.541, 6.502, 6.456, 103, -97, 6.394, 6.354, 6.328, 6.322, 6.343, 103, -97, 6.409, 6.479, 6.536, 6.581, 6.613, 6.63, 103, -97, 6.697, 6.683, 6.653, 6.606, 103, -97, 6.54, 6.456, 6.352, 6.244, 6.147, 6.062, 5.989, 5.927, 5.876, 103, -97, 5.75, 5.72, 5.701, 5.692, 5.693, 5.704, 103, -97, 5.889, 5.961, 6.061, 6.183, 6.318, 6.46, 6.6, 6.731, 6.847, 6.938, 103, -97, 7.052, 103, -97, 6.991, 6.981, 6.951, 6.907, 6.852, 103, -97, 6.78, 6.713, 6.647, 6.584, 6.529, 6.485, 6.456, 103, -97, 6.44, 6.455, 6.496, 6.557, 6.635, 6.722, 6.815, 6.908, 103, -97, 7.004, 7.081, 7.143, 7.183, 103]
        angle_list = [-60, -30, 0, 0, 0, 60, 0, 0, -60, -30, 60, 30, -60, 30]
        y_positive = [y - 3 for y in y_points]

        # x_positive = [0, 0, 0]
        # y_positive = [0, 1, 3]

        path = zip(x_points, y_positive)
        tools_cmd_vel.turn_left(90)

        with_iiwa_follow_traj(path, angle_list, ir_pub)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
