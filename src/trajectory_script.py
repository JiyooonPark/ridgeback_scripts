#!/usr/bin/env python3
import numpy as np
import time
import rospy
from nav_msgs.msg import Odometry
from trajectory_script_algorithm import *

from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose
import std_msgs.msg

if __name__ == "__main__":

    # ir_sub = rospy.Subscriber('/iiwa_ridgeback_communicaiton', Odometry, get_communication)
    rospy.init_node('trajectory_node')

    path_angle, iiwa_range_list, path_x, path_y = run_algorithm()

    ir_pub = rospy.Publisher('/iiwa_ridgeback_communicaiton/trajectory', PoseArray, queue_size=1000)
    message= PoseArray()
    pose_list = []
    
    for i in range(len(path_x)):
        p = Point()
        q = Quaternion()
        pose = Pose()

        p.x = path_x[i]
        p.y = path_y[i]

        if path_angle[i][0] == 'l':
            q.x = -float(path_angle[i][2:])
        else:
            q.x = float(path_angle[i][2:])

        pose.position = p
        pose.orientation = q

        pose_list.append(pose)

    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    message.header = h

    message.poses = pose_list

    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        ir_pub.publish(message)
        rate.sleep()


    rospy.spin()



'''
---
header: 
  seq: 205
  stamp: 
    secs: 324
    nsecs: 406000000
  frame_id: ''
poses: 
  - 
    position: 
      x: -0.852
      y: 1.888
      z: 0.0
    orientation: 
      x: 75.738872377
      y: 0.0
      z: 0.0
      w: 0.0
  - 
    position: 
      x: -0.234
      y: 1.891
      z: 0.0
    orientation: 
      x: 89.3913028001
      y: 0.0
      z: 0.0
      w: 0.0
  - 
    position: 
      x: 0.414
      y: 1.706
      z: 0.0
    orientation: 
      x: -75.2975565619
      y: 0.0
      z: 0.0
      w: 0.0
  - 
    position: 
      x: 0.618
      y: 1.883
      z: 0.0
    orientation: 
      x: -89.9243120922
      y: 0.0
      z: 0.0
      w: 0.0
  - 
    position: 
      x: 1.347
      y: 1.837
      z: 0.0
    orientation: 
      x: -77.7023071263
      y: 0.0
      z: 0.0
      w: 0.0
---



'''