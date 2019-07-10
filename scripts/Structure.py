#!/usr/bin/env python

import time
import roslib
import rospy
import actionlib
import math
from UAV import UAV
from connection import Connection

# for this structure class, we assume the UAV structure is 2D and all of the UAV
# are on the same plane
class Structure:

    def __init__(self, UAV):
        self.__UAVS = [UAV]
        self.__CON = Connection()

    def AddUAV(self, UAV):
        self.__UAVS.append(UAV)

    def GetNumOfUAV(self):
        return len(self.__UAVS)

    def StrucPoseAction(self, Pose):
        pass

    def GetCentreOfMass(self):
        pass
