#!/usr/bin/env python
from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import sys, select, termios, tty
import rospy, time
from dynamic_reconfigure.msg import Config

def get_current_position(msg):
    # follows the conventional x, y, poses
    print(msg.bools)
class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/move_base/DWAPlannerROS/parameter_updates', Config, queue_size = 1)
        self.x = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x):
        self.condition.acquire()
        self.x = x
        # print("vel:", x, y)
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0)
        self.join()

    def run(self):
        config = Config()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            config.bools = False

            self.condition.release()

            # Publish.
            self.publisher.publish(['name="hello" value=True'], [2], ['name="hello" value="ey"'], [7.6], None)

        # Publish stop message when thread exits.
        config.bools = True
        self.publisher.publish(twist)


def go_to_goal_holonomic(in_x_goal, in_y_goal):

    settings = termios.tcgetattr(sys.stdin)

    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    pub_thread = PublishThread(repeat)
    if key_timeout == 0.0:
        key_timeout = None

    odom_sub = rospy.Subscriber('/move_base/DWAPlannerROS/parameter_updates', Config, get_current_position)


    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(-10)
       
        while(goal_difference()):
            pub_thread.update(-100)
        pub_thread.stop()
    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
if __name__=="__main__":
    
    rospy.init_node('holonimoic_move_to_goal')

    go_to_goal_holonomic(-2, -2)
