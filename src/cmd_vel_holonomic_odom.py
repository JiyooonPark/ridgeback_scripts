#!/usr/bin/env python3
import tf
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

x = 0
y = 0
yaw = 0


def callback(msg):
    # follows the conventional x, y, poses
    global x, y, yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w = msg.pose.pose.orientation.w
    z = msg.pose.pose.orientation.z
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]*(180/math.pi)
    if yaw < 0:
        yaw = yaw+360
    # print(yaw)

    # print(z)
    # print(x, y)


def publish_cmd_vel():
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.linear.x = -0.05
    cmd.linear.y = 0
    cmd.linear.z = 0
    rospy.sleep(1)
    x_init = x
    y_init = y
    print("init x: {:.3f} y: {:.3f}".format(x_init, y_init))
    seconds = time.time()
    while time.time() - seconds < 20:
        publisher.publish(cmd)
    x_final = x
    y_final = y
    print("final x: {:.3f} y: {:.3f}".format(x_final,  y_final))
    print("x: {:.3f} y: {:.3f}".format(x_final - x_init,  y_final-y_init))


def publish_cmd_vel_rotate(z, sec):
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = z
    rospy.sleep(1)
    # z_init=z
    # print("init z: {:.3f}".format(  to_angle(z_init)))
    seconds = time.time()
    while time.time() - seconds < sec:
        publisher.publish(cmd)
    # z_final = z
    # print("final z: {:.3f} ".format( to_angle(z_final)))

    # print("z: {:.3f} ".format(to_angle(z_final)-to_angle(z_init)))
    print(yaw)


def angle_to_a(a):
    rospy.sleep(1)
    while abs(yaw-a) >= 0.5:
        print("diff:", yaw-a)
        publish_cmd_vel_rotate(0.1, 0.01)


def callback_laser(msg):
    print(len(msg.ranges))


def right_angle():

    from sensor_msgs.msg import LaserScan

    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, callback_laser)
    rospy.spin()


if __name__ == "__main__":

    rospy.init_node('holonimoic_move_to_goal')
    odom_sub = rospy.Subscriber('/odom', Odometry, callback)
    # go_to_goal_holonomic(2, -2)
    publish_cmd_vel()
    # publish_cmd_vel_rotate()
    # angle_to_a(110)
    # right_angle()
