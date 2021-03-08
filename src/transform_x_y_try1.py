#!/usr/bin/env python

# goal_to_goal_cmd_vel.py
# uses cmd_vel topic to publish velocity
# it takes in a goal position and re evaluates its velocity everytime amcl_pose is updated
# tip :  do not erase the run function even if it seems useless... it won't work if you delete it. 
# used teleop_twist_keyboard.py code for reference. 

from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import sys, select, termios, tty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np

angle=0
kp=0.5

global x_pose, y_pose, w_pose, z_pose 
x_pose = 0
y_pose = 0
w_pose = 0
z_pose = 0

global x_goal, y_goal
# x_goal = 0
# y_goal = 0

x_goal = 2
y_goal = 2

global x_diff, y_diff
x_diff = 0
y_diff = 0
# def callback(msg):
#     # print msg.pose.pose

def get_current_position(msg):
    global x_pose, y_pose, w_pose, z_pose
    global x_goal, y_goal
    global x_diff, y_diff

    # follows the conventional x, y, poses
    x_pose = msg.pose.pose.position.x
    y_pose = msg.pose.pose.position.y
    w_pose = msg.pose.pose.orientation.w
    z_pose = msg.pose.pose.orientation.z
    print("x:", x_pose, "y:", y_pose , "w:", w_pose)

    

    # to fix problem 
    global angle

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    angle = yaw *180 / math.pi
    if angle < 0:
        angle = angle
    else:
        pass
    print(angle)

        
    # follows the conventional x, y, poses
    x_diff = x_goal - x_pose
    y_diff = y_goal - y_pose
    a = np.array([[math.cos(angle), math.sin(angle)],[math.sin(angle), math.cos(angle)]])
    print(a)
    # x_diff_ad, y_diff_ad = np.matmul(a, [x_goal, y_goal])
    K = np.matmul(a, [x_goal, y_goal])
    print(K)

    # I want this diffs to be used in the convestional x, y axis
    print ("x_diff : ", x_diff, "y_diff : ", y_diff)


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):

        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # print("vel:", x, y)
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()
    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)



def goal_difference():
    global x_diff, y_diff
    if abs(x_diff) < 0.05 and abs(y_diff) <0.05:
        print ("reached goal")
        return False
    else:
        return True

if __name__=="__main__":
    # global x_goal, y_goal, x_diff, y_diff

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('holonimoic_move_to_goal')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    pub_thread = PublishThread(repeat)
    if key_timeout == 0.0:
        key_timeout = None

    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, get_current_position)

    x_vel = 0
    y_vel = 0
    z = 0
    th = 0
    status = 0
    speed = 0.35
    i=0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x_vel, y_vel, z, th, speed, turn)
       
        while(goal_difference()):
            x_vel = x_diff / (abs(x_diff) + abs(y_diff))
            y_vel = y_diff / (abs(x_diff) + abs(y_diff))

            # a = math.sqrt((x_goal**2 + y_goal**2) / (1 + (y_goal/x_goal - math.tan(angle))**2))
            # b = ((x_goal/y_goal)-math.tan(angle))*a
            # x_vel = a / (abs(a) + abs(b))
            # y_vel = b / (abs(a) + abs(b))
            

            z = 0
            th = 0 
            if i%100000 is 0:
                # print ("a: ", a, "b: ", b)
                print (x_vel, y_vel)
            else:
                pass
            i = i+1
            pub_thread.update(x_vel, y_vel, z, th, speed, turn)
            # print ("updated")
        pub_thread.stop()

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)