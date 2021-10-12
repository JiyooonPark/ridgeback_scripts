import rospy
from geometry_msgs.msg import Twist
import time
import math
# angle related
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


def movebase_client(w):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "/odometry/filtered"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1
    goal.target_pose.pose.position.y = 2
    
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def turn_right(degrees):
    print("right")
    z = 1/18
    sec = degrees/10 * 3
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = -z
    rospy.sleep(1)
    seconds = time.time()
    while time.time() - seconds < sec:
        publisher.publish(cmd)


def turn_left(degrees):
    print("left")
    z = 1/18
    sec = degrees/10 * 3
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = z
    rospy.sleep(1)
    seconds = time.time()
    while time.time() - seconds < sec:
        publisher.publish(cmd)


# movement related


def move_forward(speed, duration):
    print("forward")
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.linear.x = speed
    cmd.linear.y = 0
    cmd.linear.z = 0
    rospy.sleep(1)
    seconds = time.time()
    while time.time() - seconds < duration:
        publisher.publish(cmd)


def move_backward(speed, duration):
    print("backward")
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.linear.x = -speed
    cmd.linear.y = 0
    cmd.linear.z = 0
    rospy.sleep(1)
    seconds = time.time()
    while time.time() - seconds < duration:
        publisher.publish(cmd)


def move_relative(x, y, duration=5):
    print("moving to :", x, y)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    cmd = Twist()
    cmd.linear.x = x/duration
    cmd.linear.y = y/duration
    cmd.linear.z = 0

    rospy.sleep(1)

    seconds = time.time()
    while time.time() - seconds < duration:
        publisher.publish(cmd)


def move_relative_rotate(x, y, angle=5, duration=5):
    print("moving to :", x, y)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    cmd = Twist()
    cmd.linear.x = -y/duration
    cmd.linear.y = -x/duration
    cmd.linear.z = 0

    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = math.radians(angle)/duration

    rospy.sleep(1)

    seconds = time.time()
    while time.time() - seconds < duration:
        publisher.publish(cmd)
