#!/usr/bin/env python

import rospy, time
from dynamic_reconfigure.msg import Config

param = None

def callback(msg):
    global param
    print("in")
    print(msg.bools[0].name)
    param = msg
    # init_x = msg.pose.pose.orientation.x
    # init_y = msg.pose.pose.orientation.y
    # init_z = msg.pose.pose.orientation.z
    # init_w = msg.pose.pose.orientation.w
    # print (init_x, init_y, init_z, init_w)

def publish_params():
    
    new_config = Config()
    param.doubles[12].value = 4
    param.doubles[13].value = 5
    result = publisher.publish(param)
    print(result)
    print(param)
    print("pushed params")


def movebase_client(goal_x, goal_y):

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    # goal.target_pose.pose.orientation.x = init_x
    # goal.target_pose.pose.orientation.y = init_y
    # goal.target_pose.pose.orientation.z = init_z
    # goal.target_pose.pose.orientation.w = init_w
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
    # x_goal = -2
    # y_goal = 2
    x_goal = 0
    y_goal = 0
    print("here")

    try:
        print("trying")
        rospy.init_node("dynamic_reconfigure_during")
        odom_sub = rospy.Subscriber("/move_base/DWAPlannerROS/parameter_updates", Config, callback)
        print(param)
        
        publisher = rospy.Publisher('/move_base/DWAPlannerROS/parameter_updates', Config, queue_size = 10)

        # publish_params()
        while not rospy.is_shutdown():
            continue


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")