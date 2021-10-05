import math
# laser scanner
def callback_laser(msg):
    global angle_120, angle_60, angle_90, m

    # for real ridgeback
    # angle_60 = (msg.ranges[490]+msg.ranges[500]+msg.ranges[510])/3
    # angle_90 = (msg.ranges[530]+msg.ranges[540]+msg.ranges[550])/3
    # angle_120 = (msg.ranges[570]+msg.ranges[580]+msg.ranges[590])/3

    # for simulation
    angle_60 = (msg.ranges[230]+msg.ranges[240]+msg.ranges[250])/3
    angle_90 = (msg.ranges[350]+msg.ranges[360]+msg.ranges[370])/3
    angle_120 = (msg.ranges[470]+msg.ranges[480]+msg.ranges[490])/3
    m = math.sqrt(3)/2

    # print("{:.4} {:.4} {:.4}".format(angle_120, angle_90, angle_60))


def callback_odom(msg):  # odom_sub = rospy.Subscriber('/odom', Odometry, callback)
    # follows the conventional x, y, poses
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y


# odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
def callback_amcl(msg):

    # rospy.sleep(1)
    print("x: {:.3f} y: {:.3f} w: {:.3f}".format(
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w))
