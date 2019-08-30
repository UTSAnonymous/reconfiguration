#!/usr/bin/env python
import rospy
import roslib
import math
from tf.transformations import *

from geometry_msgs.msg import *

# positive yaw = anticlockwise
# negative_yaw = clockwise
def simplifiedPoseMsg(x, y, z, z_rotation):

    point = Point(x, y, z)
    quat = quaternion_from_euler(0,0,z_rotation * (math.pi/180))
    quaternion = Quaternion(quat[0],quat[1],quat[2],quat[3])

    return Pose(point,quaternion)

def convert2DListToPose(list, set_height):

    y_length = len(list)
    x_length = len(list[0])

    if x_length != y_length:
        rospy.loginfo("Starting list is not a square 2D list")
        return False

    pose_list = []

    for y in range(y_length):
        for x in range(x_length):

            if list[y,x] != '0':
                pose = simplifiedPoseMsg(x+1,y+1,set_height,0)
                pose_list.append(pose)

    return pose_list
