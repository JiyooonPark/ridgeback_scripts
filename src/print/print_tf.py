#! /usr/bin/env python
# import tf
import rospy
from tf.msg import tfMessage
import math
i = 0

def callback_tf(msg):
    global i

    tf = msg.transforms[0].transform.rotation
    angle = 2 * math.acos(tf.z)
    degree = ((angle/(math.pi*2))*360.0)

    if i%60==0:
        if msg.transforms[0].child_frame_id=="base_link":
            print(degree)
    i+=1

rospy.init_node('print_tf')
odom_sub = rospy.Subscriber('/tf', tfMessage, callback_tf)

rospy.spin()
