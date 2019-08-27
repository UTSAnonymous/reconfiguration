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
