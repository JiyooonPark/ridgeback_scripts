#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

init_x = -100
init_y = 0
init_z = 0
init_w = 0


def callback(msg):
    global init_x, init_y, init_z, init_w
    if init_x == -100:
        init_x = msg.pose.pose.orientation.x
        init_y = msg.pose.pose.orientation.y
        init_z = msg.pose.pose.orientation.z
        init_w = msg.pose.pose.orientation.w
        print (init_x, init_y, init_z, init_w)
    # print (msg.pose.pose)


def movebase_client(goal_x, goal_y):

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    for i in range(5):
        print(i)
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = i
        goal.target_pose.pose.position.y = i
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            continue

if __name__ == "__main__":
    # x_goal = -2
    # y_goal = 2
    x_goal = 0
    y_goal = 0

    try:
        rospy.init_node("movebase_client_py")
        odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
        result = movebase_client(x_goal, y_goal)
        if result:
            print (init_x, init_y, init_z, init_w)
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")