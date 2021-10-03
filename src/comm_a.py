#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String

iiwa = 0

def get_communication(msg):
    global iiwa
    iiwa = int(msg.data)
    print(f'iiwa said: {msg.data}')


if __name__ == '__main__':
    try:
        rospy.init_node('comm_a')


        rospy.Subscriber('/iiwa_ridgeback_communicaiton/iiwa', String, get_communication)
        ir_pub = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback', String, queue_size=100)
        i = 1
        rate = rospy.Rate(10)
        while i < 10:
            
            send = input('input:')
            ir_pub.publish(send)
            while iiwa != i+1:
                time.sleep(1)
            i += 1
        rospy.sleep(0.5)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
