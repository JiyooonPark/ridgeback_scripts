#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

graph_limit = 3

# linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = 0,0,0,0,0,0 
linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = [],[],[],[],[],[]
i=0
def get_current_position(msg):
    global linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, i
    # follows the conventional x, y, poses
    pose_position = msg.pose.pose.position
    position_x = pose_position.x
    position_y = pose_position.y
    position_z = pose_position.z
    i += 1

    # if len(linear_x) >= 15:
    #     linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = [],[],[],[],[],[]
    if i%10 ==0:
        linear_x.append(round(position_x,3))
        linear_y.append(round(position_y,3))
        linear_z.append(round(position_z,3))
        # linear_x = round(twist_linear.x,3)
        # linear_y = round(twist_linear.y,3)
        # linear_z = round(twist_linear.z,3)
    print(linear_x)


import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.gca(projection='3d')

def animate(i):
    ax.clear()
    ax.set_xlim(-graph_limit, graph_limit)
    ax.set_ylim(-graph_limit, graph_limit)
    ax.set_zlim(-graph_limit, graph_limit)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Tracker Pose')
    ax.plot3D(linear_x, linear_y, linear_z)
rospy.init_node('print_tracker_pose')
odom_sub = rospy.Subscriber('/vive/LHR_0B028308_pose', PoseWithCovarianceStamped, get_current_position)


ani = animation.FuncAnimation(fig, animate, interval=10)

plt.show()
# rospy.spin()