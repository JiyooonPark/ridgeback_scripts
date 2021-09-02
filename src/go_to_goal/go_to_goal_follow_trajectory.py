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


def movebase_client(goal_x, goal_y):

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    # goal.target_pose.pose.orientation.x = 0
    # goal.target_pose.pose.orientation.y = 0
    # goal.target_pose.pose.orientation.z = 0
    # goal.target_pose.pose.orientation.w = 0.5

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == "__main__":

    try:
        rospy.init_node("movebase_client_py")
        odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
        x_points = [-3.08, -1.61, -0.24, 0.11, 1.81, 3.72, 4.51, 4.69, 6.04, 6.84, 8.16, 8.78, 9.15, 10.05]
        y_points = [-5.66, -4.93, -6.1, -5.54, -4.42, -4.03, -6.07, -5.22, -4.1, -2.57, -3.06, -5.01, -5.23, -6.93]
        y_positive = [abs(y)-0.5 for y in y_points]
        for (x, y) in zip(x_points, y_positive):

            result = movebase_client(x, y)
            if result:
                print (init_x, init_y, init_z, init_w)
                rospy.loginfo("Goal execution done!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")