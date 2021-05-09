#!/usr/bin/env python
import tf
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import time, math
x=0
y=0
def callback(msg):
    # follows the conventional x, y, poses
    global x, y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # print(x, y)

def publish_cmd_vel():
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    cmd = Twist()
    # always set to 0.05 for optimal result
    cmd.linear.x = 0
    cmd.linear.y = -0.05
    cmd.linear.z = 0
    rospy.sleep(1)
    x_init = x
    y_init = y
    print("init x: {:.3f} y: {:.3f}".format( x_init,y_init))
    seconds = time.time()
    for i in range(5):
        print "in ",str(i+1),"st loop"
        
        cmd.linear.x = -cmd.linear.x
        cmd.linear.y = -cmd.linear.y
        cmd.linear.z = -cmd.linear.z
        
        while time.time() - seconds <2:
            # print("in while loop")
            publisher.publish(cmd)
        seconds = time.time()    
        x_final = x
        y_final = y
        # print("final x: {:.3f} y: {:.3f}".format( x_final,  y_final))
        # print("x: {:.3f} y: {:.3f}".format( x_final - x_init,  y_final-y_init))

if __name__=="__main__":
    
    rospy.init_node('holonimoic_move_to_goal')
    odom_sub = rospy.Subscriber('/odom', Odometry, callback)
    publish_cmd_vel()
