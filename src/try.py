#!/usr/bin/env python3
from tools import tools_cmd_vel
import math
import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import MultiArrayLayout


def get_communication(msg):
    global received_message
    msg_data = msg.split()


if __name__ == '__main__':
    try:
        rospy.init_node('drawing_ridgeback')

        msg = PoseArray()

        cmd_vel_topic = '/hello'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, PoseArray, queue_size=10)
        # msg.header = 1
        i=0
        j = 0
        
        while (True):
            p = Pose()
            list_pose = []
            for i in range(10):
                p.position.x, p.position.y = i, j 
                q = Quaternion()
                q.x = -1
                p.orientation = q
                list_pose.append(p)
                i += 1
                j += 1
            msg.poses = list_pose
            velocity_publisher.publish(msg)
            


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
