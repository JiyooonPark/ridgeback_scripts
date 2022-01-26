#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time, math

# ============= VARIABLES ============== #
graph_limit = 3

class Tracker:

    def __init__(self):
        self.pose_list = {}
        self.i = 0
        self.transform_vector = []
        self.R_M = None
        self.orientation = []
        self.pose=[]
     
    def quaternion_rotation_matrix(self):

        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """

        Q = self.orientation

        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix


class Ridgeback:

    def __init__(self):
        self.tracker_name = '515D3307'
        # self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_pose = rospy.Publisher('/vive_pose/filtered', Odometry, queue_size=1)
        self.subscriber_vive = rospy.Subscriber('/vive/LHR_'+self.tracker_name+'_pose', PoseWithCovarianceStamped, self.get_current_position)
        
        self.linear_speed = 0.1
        self.reached = False
        self.linear_x, self.linear_y, self.linear_z = [],[],[]
        self.tracker = Tracker()
        self.linear_vx, self.linear_vy, self.linear_vz = [],[],[]
        self.timestamps={'step0':10, 'step1':50, 'step2':100}


    def get_current_position(self,msg):

        pose_position = msg.pose.pose.position
        position_x = pose_position.x
        position_y = pose_position.y
        position_z = pose_position.z
        self.tracker.pose = [position_x, position_y, position_z]

        pose_orientation = msg.pose.pose.orientation
        orientation_w = pose_orientation.w
        orientation_x = pose_orientation.x
        orientation_y = pose_orientation.y
        orientation_z = pose_orientation.z

        self.tracker.orientation = [orientation_w, orientation_x, orientation_y, orientation_z]

        self.tracker.i+=1
        
        if self.timestamps['step0'] == self.tracker.i:
            self.tracker.pose_list['0'] = [self.tracker.pose, self.tracker.orientation]
            print('DONE MOVING 1ST')
            self.move_relative(0, -0.3)

            self.tracker.pose_list['1'] = [self.tracker.pose, self.tracker.orientation]
            print('DONE MOVING 2ND')
            self.move_relative(0.3, 0)

            self.tracker.pose_list['2'] = [self.tracker.pose, self.tracker.orientation]
            print('DONE MOVING')
            rospy.sleep(10)
        
        R_q = self.tracker.quaternion_rotation_matrix()

        x_b_hat = np.array(self.tracker.pose_list['1'])- np.array(self.tracker.pose_list['0'])
        y_b_hat = np.array(self.tracker.pose_list['2'])- np.array(self.tracker.pose_list['1'])
        z_b_hat = np.cross(x_b_hat, y_b_hat)
        
        x_b_hat = np.dot(R_q, x_b_hat)
        y_b_hat = np.dot(R_q, y_b_hat)
        z_b_hat = np.dot(R_q, z_b_hat)
        
        x_a_hat, y_a_hat , z_a_hat = [1,0,0], [0,1,0], [0,0,1]

        self.R_ba = np.array([
            [np.dot(x_b_hat, x_a_hat), np.dot(y_b_hat, x_a_hat), np.dot(z_b_hat, x_a_hat)],
            [np.dot(x_b_hat, y_a_hat), np.dot(y_b_hat, y_a_hat), np.dot(z_b_hat, y_a_hat)],
            [np.dot(x_b_hat, z_a_hat), np.dot(y_b_hat, z_a_hat), np.dot(z_b_hat, z_a_hat)]
        ])

        # angle = 2*math.acos(self.tracker.orientation[0])
        x = self.tracker.orientation[1]/math.sqrt(1-self.tracker.orientation[0]**2)
        y = self.tracker.orientation[2]/math.sqrt(1-self.tracker.orientation[0]**2)
        z = self.tracker.orientation[3]/math.sqrt(1-self.tracker.orientation[0]**2)

        orientation = np.dot(R_q, [x,y,z])

        world_frame = np.dot(R_q, self.tracker.pose)
        world_frame = np.dot(self.R_ba, world_frame)

        # pose = np.dot(self.R_ba, self.tracker.pose)

        if self.tracker.i > self.timestamps['step2']:
            self.linear_vz.append(world_frame[0])
            self.linear_vy.append(world_frame[1])
            self.linear_vx.append(world_frame[2])

            odom_msgs = Odometry()
            odom_msgs.pose.pose.position.x = world_frame[0]
            odom_msgs.pose.pose.position.y = world_frame[1]
            odom_msgs.pose.pose.position.z = world_frame[2]

            odom_msgs.pose.pose.orientation.x = orientation[0]
            odom_msgs.pose.pose.orientation.y = orientation[1]
            odom_msgs.pose.pose.orientation.z = orientation[2]

            self.publisher_pose.publish(odom_msgs)

            print(f'x: {round(world_frame[2],3)}, y:{round(world_frame[1],3)}')

    def check_speed(self, speed):
        if speed >= 0.03:
            return 0.03
        elif speed <= -0.03:
            return -0.03
        else:
            return speed

    def move_relative(self, x, y, duration=5):
        print("moving to :", x, y)

        cmd = Twist()
        cmd.linear.x = self.check_speed(x/duration)
        cmd.linear.y = self.check_speed(y/duration)
        cmd.linear.z = 0

        rospy.sleep(1)

        seconds = time.time()
        while time.time() - seconds < duration:
            self.publisher_cmd_vel.publish(cmd)

    def animate(self, i):
        ax.clear()
        ax.set_xlim(-graph_limit, graph_limit)
        ax.set_ylim(-graph_limit, graph_limit)
        ax.set_zlim(-graph_limit, graph_limit)

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        ax.set_title('Tracker Pose')

        ax.plot3D(self.linear_x, self.linear_y, self.linear_z, color='r')
        ax.plot3D(self.linear_vx, self.linear_vy, self.linear_vz, color='b')

    def start(self):
        ani = animation.FuncAnimation(fig, self.animate, interval=10)
        plt.show()

# def plot_vector(vector, color):
#     ax.plot3D([0,vector[0]], [0,vector[1]], [0,vector[2]], color)


if __name__=='__main__':

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    rospy.init_node('print_tracker_pose')

    Rid = Ridgeback()
    Rid.start()
    