#!/usr/bin/env python3

import rospy
from tools import tools_cmd_vel
import time
from geometry_msgs.msg import Twist


def get_tracker_pose(msg):
    pass
    

def move_relative(x, y, duration=5):
    print("moving to :", x, y)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    cmd = Twist()
    cmd.linear.x = x/duration
    cmd.linear.y = y/duration
    cmd.linear.z = 0

    rospy.sleep(1)

    seconds = time.time()
    while time.time() - seconds < duration:
        publisher.publish(cmd)

if __name__ == "__main__":

    # goal x,y : the final goal
    x_goal = -1
    y_goal = 0

    try:
        rospy.init_node('relative_move')
        rospy.loginfo("moving to goal")
        tools_cmd_vel.move_relative(x_goal, y_goal, duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")

