#!/usr/bin/env python

from nav_msgs.msg import Odometry
import rospy
from geometry_msgs.msg import Twist
import math
import time
from std_srvs.srv import Empty
x = 0
y = 0
yaw = 0


def print_pose(msg):
    global x, y, yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = msg.twist.twist.angular.z  # yaw!!!!!!!!!!?????
    # print(msg.pose.pose)


def move(speed, distance, is_forward):
    # declare a Twist message to send velocity commands
    velocity_message = Twist()
    # get current location

    if (speed > 0.4):
        print('speed must be lower than 0.4')
        return

    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)
    print('vel is ', velocity_message)

    distance_moved = 0.0
    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(100)
    cmd_vel_topic = 'cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    # t0 = rospy.get_time()
    t0 = time.time()
    print("init time", t0)

    # time_needed = distance/speed
    # print(time_needed)
    # needed_seconds = Duration(time_needed)

    i = 0
    while True:
        i = i+1
        rospy.loginfo("Turtlesim moves forwards")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()
        # t1 =  rospy.get_time()
        t1 = time.time()
        if i % 100 is 0:
            print(t1)
        # rospy.Duration(1.0)

        distance_moved = (t1-t0) * speed
        print(distance_moved)
        if not (distance_moved < distance):
            rospy.loginfo("reached")
            break

    # finally, stop the robot when the distance is moved
    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)


def rotate(angular_speed_degree, relative_angle_degree, clockwise):

    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    angular_speed = math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)

    angle_moved = 0.0
    # we publish the velocity at 10 Hz (10 times a second)
    loop_rate = rospy.Rate(10)
    cmd_vel_topic = '/cmd_vel_mux/input/teleop'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        print('current_angle_degree: ', current_angle_degree)

        if (current_angle_degree > relative_angle_degree):
            rospy.loginfo("reached")
            break

    # finally, stop the robot when the distance is moved
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)


def go_to_goal(x_goal, y_goal):
    global x
    global y, yaw

    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)

    i = 0

    K_angular = 4.0
    desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
    angular_speed = (desired_angle_goal-yaw)*K_angular
    velocity_message.angular.z = angular_speed

    while (True):
        i = i+1

        K_linear = 0.5
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        velocity_message.angular.z = 0

        # print ('x=', x, 'y=',y)
        if i % 300 is 0:
            print('distance', distance)

        else:
            continue

        if (distance < 0.01):
            print('done')
            break


if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        listener = rospy.Subscriber('/odom', Odometry, print_pose)
        # move (0.2, 1 , True)
        time.sleep(1.0)
        go_to_goal(2, 3)
        # rotate (90, 90 , True)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
