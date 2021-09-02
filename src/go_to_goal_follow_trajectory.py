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
        # print (init_x, init_y, init_z, init_w)
    # print (msg.pose.pose)


def movebase_client(goal_x, goal_y):

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
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
        return client.get_result()


if __name__ == "__main__":

    x_points = [-1.65, -1.25, -0.88, -0.34, 0.13, 0.64, 1.19, 1.78, 2.0, 2.58, 3.05, 3.39, 3.96, 4.33, 4.74, 5.24, 5.86, 6.09, 6.36, 6.76, 7.29, 7.72, 8.21, 8.88, 9.02, 9.49, 9.96]
    y_points = [-4.75, -4.99, -5.19, -5.2, -5.11, -5.02, -5.0, -5.1, -5.16, -5.28, -5.25, -5.11, -4.76, -4.47, -4.37, -4.39, -4.76, -5.38, -5.53, -5.54, -5.41, -5.15, -5.02, -5.08, -5.14, -5.59, -5.75]

    x_positive = [x+1.3 for x in x_points]
    y_positive = [abs(y)-1.2 for y in y_points]

    try:
        rospy.init_node("movebase_client_py")
        odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
        for x_goal, y_goal in zip(x_positive, y_positive):
            result = movebase_client(x_goal, y_goal)
            if result:
                print (init_x, init_y, init_z, init_w)
        
        rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")