#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
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
    # print('in function' ,  x, y, w)
    return x, y, w


def movebase_client(x_goal, y_goal):
    global x, y, w
    # rospy.init_node('check_odometry')
    # odom_sub = rospy.Subscriber('/odom', Odometry, callback)
    # rospy.spin()

    while not rospy.is_shutdown():
        print(x, y, w)

    x_move = x_goal - x
    y_move = y_goal - y

    # print(x_move, y_move)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def go_to_goal(x_goal, y_goal):
    global x
    global y, w

    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)
    print('current position: ', x, y)

    i = 0
    while (True):
        i = i+1

        K_linear = 0.05
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
        if i % 300000 == 0:
            print('distance', distance)
            print(x, y)

        else:
            continue

        if (distance < 0.05):
            print('done')
            break


if __name__ == '__main__':
    try:
        rospy.init_node('move_to_fixed_pose')
        odom_sub = rospy.Subscriber('/odom', Odometry, get_current_position)
        rospy.sleep(0.5)
        # print(x, y,w)
        result = go_to_goal(0, 0)
        if result:
            rospy.loginfo("Goal execution done!")
            odom_sub = rospy.Subscriber(
                '/odom', Odometry, get_current_position)
            rospy.sleep(0.5)
            print(x, y, w)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
