#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import time

x=0
y=0
def get_current_position(msg):
    # follows the conventional x, y, poses
    global x, y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # print(x, y)

def publish_cmd_vel():
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    cmd = Twist()
    cmd.linear.x = 0.05
    cmd.linear.y = 0
    cmd.linear.z = 0
    rospy.sleep(1)
    x_init = x
    y_init = y
    print("init x: {:.3f} y: {:.3f}".format( x_init,y_init))
    seconds = time.time()
    while time.time() - seconds <10:
        publisher.publish(cmd)
    x_final = x
    y_final = y
    print("final x: {:.3f} y: {:.3f}".format( x_final,  y_final))
    print("x: {:.3f} y: {:.3f}".format( x_final - x_init,  y_final-y_init))

if __name__=="__main__":
    
    rospy.init_node('holonimoic_move_to_goal')
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, get_current_position)
    # go_to_goal_holonomic(2, -2)
    publish_cmd_vel()
