#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String

ridgeback = 0

def get_communication(msg):
    global ridgeback
    ridgeback = int(msg.data)
    print(f'ridgeback said: {msg.data}')


if __name__ == '__main__':
    try:
        rospy.init_node('comm_b')


        rospy.Subscriber('/iiwa_ridgeback_communicaiton/ridgeback', String, get_communication)
        ir_pub = rospy.Publisher('/iiwa_ridgeback_communicaiton/iiwa', String, queue_size=100)
        rate = rospy.Rate(10)
        i = 0
        while i < 10:
            while ridgeback != i+1:
                time.sleep(1)
            i += 1
            send = input('input:')
            ir_pub.publish(send)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
