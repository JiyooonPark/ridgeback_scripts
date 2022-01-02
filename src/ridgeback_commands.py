#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import math, time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import *
from trajectory_script_algorithm import *

class Ridgeback:

    def __init__(self):

        self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subscriber_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odom)
        self.linear_speed = 0.1
        self.reached = False
        self.i = 0

    def euler_from_quaternion(self, orientation_list):

        [x, y, z, w] = orientation_list
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 

    def callback_odom(self, msg):

        # get position
        pose_position = msg.pose.pose.position
        self.position_x = pose_position.x
        self.position_y = pose_position.y

        # get orientation
        self.orientation_q = msg.pose.pose.orientation
        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]

        (roll, pitch, self.yaw) = self.euler_from_quaternion(orientation_list)
        if self.yaw<0:
            angle = -self.yaw*180/math.pi
        else:
            angle = 360-self.yaw*180/math.pi

        self.rad = math.radians(angle)

    def fixed_goal_time(self, goal_x, goal_y):

        x_move = goal_x-self.position_x
        y_move = goal_y-self.position_y

        goal = np.array([[x_move],[y_move]])

        rotation_matrix = np.array([[math.cos(self.rad), -math.sin(self.rad)], 
                                    [math.sin(self.rad), math.cos(self.rad)]])

        r_goal = np.matmul(rotation_matrix, goal)

        self.move_relative(float(r_goal[0][0]), float(r_goal[1][0]), duration=10)

    def move_relative(self, x, y, duration=5):
        # change this to not use time

        print(f"Moving to: {x, y}")

        cmd = Twist()
        cmd.linear.x = x/duration
        cmd.linear.y = y/duration

        seconds = time.time()
        while time.time() - seconds < duration:
            self.publisher_cmd_vel.publish(cmd)

    def fixed_goal(self, goal_x, goal_y, duration=5):

        self.publish_state('0')
        cmd = Twist()

        while not self.reached:

            x_move = goal_x-self.position_x
            y_move = goal_y-self.position_y
            self.i +=1
            if self.i % 300000 == 0:
                print("distance: ", dist)
            goal = np.array([[x_move],[y_move]])

            rotation_matrix = np.array([[math.cos(self.rad), -math.sin(self.rad)], 
                                        [math.sin(self.rad), math.cos(self.rad)]])

            r_goal = np.matmul(rotation_matrix, goal)
            dist = self.calculate_distance(r_goal)
            
            if not self.reached:
                if dist > 0.3:
                    cmd.linear.x = self.linear_speed * r_goal[0][0]
                    cmd.linear.y = self.linear_speed * r_goal[1][0]

                if dist < 0.3:
                    cmd.linear.x = 0
                    cmd.linear.y = 0
                    self.reached = True
                    print('DONE')
        self.reached = False

    def calculate_distance(self, r_goal):
        return math.sqrt(r_goal[0]**2+ r_goal[1]**2)

    def fixed_rotate(self, target_angle):

        kp=0.5
        command =Twist()

        if 0<=target_angle<=180:
            real_target = -target_angle
        else:
            real_target = 360-target_angle

        while True:
            target_rad = real_target*math.pi/180
            command.angular.z = kp * (target_rad-self.yaw)
            self.publisher_cmd_vel.publish(command)

            if abs(self.yaw*180/math.pi-real_target)<1:
                break

    def callback_laser(self, msg):
        # for real ridgeback
        # self.angle_60 = (msg.ranges[490]+msg.ranges[500]+msg.ranges[510])/3
        # self.angle_90 = (msg.ranges[530]+msg.ranges[540]+msg.ranges[550])/3
        # self.angle_120 = (msg.ranges[570]+msg.ranges[580]+msg.ranges[590])/3

        # for simulation
        self.angle_60 = (msg.ranges[230]+msg.ranges[240]+msg.ranges[250])/3
        self.angle_90 = (msg.ranges[350]+msg.ranges[360]+msg.ranges[370])/3
        self.angle_120 = (msg.ranges[470]+msg.ranges[480]+msg.ranges[490])/3

    def right_angle(self):
        thresh = 0.01
        rospy.sleep(1)
        while True:
            if self.angle_120 - self.angle_60 > thresh:
                self.fixed_rotate(self.yaw + 0.05)
            elif self.angle_120 - self.angle_60 < -thresh:
                self.fixed_rotate(self.yaw - 0.05)
            else:
                print(f'60: {self.angle_60}, 120: {self.angle_120}, 120-60: {self.angle_120-self.angle_60}')
                print("DONE")
                break
if __name__ == '__main__':

    rospy.init_node('RIDGEBACK', anonymous=True)

    Rid = Ridgeback()