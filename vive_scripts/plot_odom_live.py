#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry

linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = 0,0,0,0,0,0 #
linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = [],[],[],[],[],[]
i=0
def get_current_position(msg):
    global linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, i
    # follows the conventional x, y, poses
    twist_linear = msg.twist.twist.linear
    twist_angular = msg.twist.twist.angular
    i += 1

    if len(linear_x) >= 200:
        linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = [],[],[],[],[],[]
    if i%10 ==0:
        linear_x.append(round(twist_linear.x,3))
        linear_y.append(round(twist_linear.y,3))
        linear_z.append(round(twist_linear.z,3))
        # linear_x = round(twist_linear.x,3)
        # linear_y = round(twist_linear.y,3)
        # linear_z = round(twist_linear.z,3)
        angular_x = twist_angular.x
        angular_y = twist_angular.y
        angular_z = twist_angular.z
    print(linear_x)


import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from mpl_toolkits.mplot3d import Axes3D
style.use('fivethirtyeight')

fig = plt.figure()
ax = plt.axes(projection='3d')
# ax = fig.add_subplot(111,projection='3d') 

limits_x = [-3, -3, -3, -3, 3, 3, 3, 3]
limits_y = [-3, -3, 3, 3, -3, -3, 3, 3]
limits_z = [-3, 3, -3, 3,-3, 3, -3, 3]

def animate(i):
    # ax.set_xlim(-2,2)
    # ax.set_ylim(-2,2)
    # ax.set_zlim(-2,2)
    # ax.set_xticks([-2, -1, 0, 1, 2])
    # ax.set_yticks([-2, -1, 0, 1, 2])
    # ax.set_zticks([-2, -1, 0, 1, 2])

    # ax.axes.set_xlim3d(left=0.2, right=2) 
    # ax.axes.set_ylim3d(bottom=-2, top=2) 
    # ax.axes.set_zlim3d(bottom=-2, top=2) 

    print('animate!')

    ax.clear()
    ax.scatter3D(limits_x, limits_y, limits_z, c='b', linewidth=0.01)
    ax.plot3D(linear_x, linear_y, linear_z)
    # ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5);
rospy.init_node('print_tracker_pose')
odom_sub = rospy.Subscriber('/vive/LHR_2FCD60B0_odom', Odometry, get_current_position)

ani = animation.FuncAnimation(fig, animate, interval=10)

plt.show()
rospy.spin()