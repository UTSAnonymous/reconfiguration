#!/usr/bin/env python
import time
from Structure import Structure
from UAV import UAV

from geometry_msgs.msg import *
from hector_uav_msgs.msg import *

#TODO:
'''
1. Create a structure class which takes an input of UAV list/ number of drones
2. In the constructure then (maybe) create individual UAV object and insert into a Structure
- The structures class consist of 1 or multiple structure class
3. Consist of function which breaks or connect individual/ or multiple UAV
'''
class Structures():

    def __init__(self, number_of_uav):
        self.__structure = {}

        '''
        1. Create the correct number of UAV object
        2. Move each of them to a correct flying location
        3. Put all into a single structure object
        4. Append the single structure into self.__structures
        '''
        # create the correct number of UAV
        uavs = []
        for i in range(1, number_of_uav+1):
            uavs.append(UAV("uav"+str(i)))

        # position the UAV in the initial starting position
        x_delta = 1
        y_delta = 1
        x = 0
        y = 0
        z = 5
        for i in range(number_of_uav):
            pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))

            # fly to this position
            uavs[i].takeoffAction()
            uavs[i].poseAction(pose)

            time.sleep(1)

            x = x + x_delta
            if i%4 == 0 and i != 0:
                y = y + y_delta
                x = 0

        for uav in uavs:
            uav.poseActionWaitClient(50)

        # Insert all of the UAV into a single structure
        structure = Structure(uavs[0])

        for i in range(1,number_of_uav):
            structure.connectSingleUAV(uavs[i])

        # append single structure into self.__structures
        self.__structures.append(structure)

    def connectUAV(self, uavs):
        '''
        uavs is a list
        '''
        pass


    def disconnectUAV(self):
        pass

    def structureGoToPose(self, structure, pose):
        pass
