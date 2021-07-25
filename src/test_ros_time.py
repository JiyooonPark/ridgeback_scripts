#! /usr/bin/env python3
import rospy
import time


rospy.init_node('rostime')

# now = rospy.Time.from_sec(time.time()).to_sec
# # now = rospy.Duration.from_sec(1)
# print("now : ", str(now))


# print(rospy.Time()) # 0
# print(rospy.get_rostime())
# print(rospy.Time.now())
# rospy.spin()

# start = rospy.get_time()
# print(start)
# rospy.sleep(2)

# print(rospy.get_time())
# print(rospy.get_time() - start)
# while rospy.sleep(1):
#     print("hey")

seconds = time.time()
print("Seconds since epoch =", seconds)
while time.time() - seconds < 1:
    pass
print("Seconds since epoch =", time.time())
