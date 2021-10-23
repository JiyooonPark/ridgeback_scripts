#!/usr/bin/env python3
from tools import tools_cmd_vel
import math
import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

x = 0
y = 0
w = 0
PI = math.pi


received_message = {}
send_message = {}


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
                tools_cmd_vel.turn_right(angle_list[angle_index])
            elif angle_list[angle_index] == 0:
                pass
            else:
                tools_cmd_vel.turn_left(-angle_list[angle_index])
            print('drawing')
            time.sleep(16)
            
            continue
        elif px <- 90 and py > 90:
            if angle_list[angle_index] > 0:
                tools_cmd_vel.turn_left(angle_list[angle_index])
            elif angle_list[angle_index] == 0:
                pass
            else:
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
        x_points = [-0.864, 0.246, 1.624, 2.834, 3.329, 4.322, 5.604, 6, 6.465, 7.417, 8.433, 9.663]
        y_points = [5.008, 4.909, 4.932, 5.109, 4.803, 4.293, 4.325, 4.3, 5.331, 5.089, 4.837, 5.493]
        
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
