#!/usr/bin/env python
import tf
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import time, math

yaw=0
def callback(msg):
    # follows the conventional x, y, poses
    global x, y,yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]*(180/math.pi)
    if yaw<0:
        yaw = yaw+360
    # print(yaw)

def publish_cmd_vel_rotate(z, sec):
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    cmd = Twist()
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = z
    rospy.sleep(1)
    seconds = time.time()
    while time.time() - seconds <sec:
        publisher.publish(cmd)
    print(yaw)

def angle_to_a(a):
    rospy.sleep(1)
    while abs(yaw-a)>=0.5:
        print("diff:", yaw-a)
        publish_cmd_vel_rotate(0.1, 0.01)

def callback_laser(msg):
    print len(msg.ranges)

if __name__=="__main__":
    
    rospy.init_node('holonimoic_move_to_goal')
    odom_sub = rospy.Subscriber('/odom', Odometry, callback)
    publish_cmd_vel_rotate()
