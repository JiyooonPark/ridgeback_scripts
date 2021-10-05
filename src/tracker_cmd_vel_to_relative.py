#!/usr/bin/env python3

import rospy
from tools import tools_cmd_vel
from nav_msgs.msg import Odometry

init_pose = []
final_pose = []
i = 0

def callback(msg):
    global i, final_pose, init_pose
    
    pose = msg.pose.pose.position
    if i < 5:
        print(f'init pose: {round(pose.x, 3)}, {round(pose.y, 3)}')
        init_pose = [round(pose.x, 3), round(pose.y, 3)]

    i+=1
    if i % 40 == 0:
        print(f'pose: {round(pose.x, 3)}, {round(pose.y, 3)}')
        final_pose = [round(pose.x, 3), round(pose.y, 3)]

if __name__ == "__main__":
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, callback)

    # goal x,y : the final goal
    x_goal = 0
    y_goal = -1

    try:
        rospy.init_node('cmd_vel_go_to_goal')
        tools_cmd_vel.move_relative(x_goal, y_goal, duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")
        
    print(f'init pose: {init_pose}\nfinal pose:{final_pose}')
    print(f'variation: {final_pose[0] - init_pose[0]}, {final_pose[1] - init_pose[1]}')

