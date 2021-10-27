#! /usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy
from geometry_msgs.msg import Twist
from tools import tools_cmd_vel
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

init_pose = []
final_pose = []
i = 0

def callback(msg):
    global i, final_pose, init_pose, position_x, position_y, position_z
    pose_position = msg.pose.pose.position
    position_x = pose_position.x
    position_y = pose_position.y
    position_z = pose_position.z

    # pose_orientation = msg.pose.pose.orientation
    # orientation_x = pose_orientation.x
    # orientation_y = pose_orientation.y
    # orientation_z = pose_orientation.z
    if i < 5:
        print(f'init pose: {round(position_x, 3)}, {round(position_y, 3)}')
        init_pose = [position_x, position_z]

    i+=1
    # if i % 80 == 0:
        # print(f'pose: {round(pose.x, 3)}, {round(pose.y, 3)}')
    final_pose = [position_x, position_z]

angle_90 = 0
angle_60 = 0
angle_120 = 0

def callback_laser(msg):
    global angle_120, angle_60, angle_90

    # for real ridgeback
    angle_60 = (msg.ranges[490]+msg.ranges[500]+msg.ranges[510])/3
    angle_90 = (msg.ranges[530]+msg.ranges[540]+msg.ranges[550])/3
    angle_120 = (msg.ranges[570]+msg.ranges[580]+msg.ranges[590])/3

    # # for simulation
    # angle_60 = (msg.ranges[230]+msg.ranges[240]+msg.ranges[250])/3
    # angle_90 = (msg.ranges[350]+msg.ranges[360]+msg.ranges[370])/3
    # angle_120 = (msg.ranges[470]+msg.ranges[480]+msg.ranges[490])/3



def right_angle():
    thresh = 0.01
    rospy.sleep(1)
    while True:
        if angle_120 - angle_60 > thresh:
            tools_cmd_vel.turn_right(0.05)
        elif angle_120 - angle_60 < -thresh:
            tools_cmd_vel.turn_left(0.05)
        else:
            print(angle_60, angle_120, angle_120-angle_60)
            print("done?")
            break


def go_to_goal(goal_distance, right):

    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    i = 0
    while (True):
        i += 1
        if i%20 == 0:
            print(f'distance: {round(distance, 3)}')

        K_linear = 0.05
        x_dist = init_pose[0] - position_x
        z_dist = init_pose[1] - position_z
        distance = math.hypot(x_dist, z_dist)
        if right:
            direction = -1
        else:
            direction = 1

        if distance < goal_distance:
            velocity_message.linear.y = K_linear * direction
        else:
            velocity_message.linear.y = -K_linear * direction

        velocity_publisher.publish(velocity_message)

        if (distance < 0.01):
            print('done')


if __name__ == "__main__":
    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/front/scan', LaserScan,callback_laser)
    odom_sub = rospy.Subscriber('/vive/LHR_BD4ED973_pose', PoseWithCovarianceStamped, get_current_position)
    right_angle()

    odom_sub = rospy.Subscriber('/odom', Odometry, callback)

    # goal x,y : the final goal
        
    print(f'init pose: {init_pose}\nfinal pose:{final_pose}')
    try:
        rospy.sleep(1)
        go_to_goal(0.3)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    distance = ((((start[0] - end[0] )**2) + ((start[1]-end[1])**2) )**0.5)
    print(distance)