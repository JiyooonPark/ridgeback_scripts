#!/usr/bin/env python

import rospy
from dynamic_reconfigure.msg import Config

def callback(msg):
    print("in")
    print("xy_goal_tolerance: ", msg.doubles[12].value)
    print("yaw_goal_tolerance: ", msg.doubles[13].value)


if __name__ == "__main__":
    print("here")

    try:
        print("trying")
        rospy.init_node("dynamic_reconfigure_py")
        odom_sub = rospy.Subscriber("/move_base/DWAPlannerROS/parameter_updates", Config, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Done")