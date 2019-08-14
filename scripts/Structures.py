#!/usr/bin/env python
import time
import rospy
import roslib
from Structure import Structure
from UAV import UAV
from connection import Connection

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
        self.__structure = []
        self.__CON = Connection()

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
        self.__structure.append(structure)

    def connectUAV(self, uavs):
        '''
        uavs is a nested list of uav names
        '''
        pass


    def disconnectUAV(self, uavs_name_list):
        '''
        UAV is a list of list of uav names
        0. convert uav name list to uav object list
        1. Check if the number of uavs in the list are the same in the structure
        2. Check if uav in the nested list is connected
        3. Figure out the correct joint to destory
        '''

        # create a list of uav objects similar to uav_name_list
        uavs_list = []
        existing_UAV = existing_UAV = self.__structure[0].getUAVList()

        for group_list in uavs_name_list:
            group = []

            for uav_input in group_list:
                for uav in existing_UAV:
                    if uav.model_name == uav_input:
                        group.append(uav)

            uavs_list.append(group)

        # check if the number of uav is the same in the struct and list
        number_of_uav = 0
        for i in range(len(uavs_list)):
            number_of_uav += len(uavs_list[i])

        rospy.loginfo(number_of_uav)
        rospy.loginfo(self.__structure[0].getNumOfUAV())

        if self.__structure[0].getNumOfUAV() != number_of_uav:
            rospy.loginfo("Number of uav disconnection is not the same as number of uav in the structure")
            return False

        # check if the uav in each group is connected
        for group in uavs_list:
            # the 2 for loop is to allow for checking the uav within the nested list
            for i in range(len(group)):
                for uav in group:
                    if group[i].model_name != uav.model_name:
                        if group[i].checkUAVConnection(uav) == False:
                            rospy.loginfo("Incorrect disconnection list!")
                            return False

        # figure out disconnection
        number_of_structure = len(uavs_list)
        # this for loop select which list to disconnect
        for selected in range(len(uavs_list)):
            # this for loop select the other list to disconnect from the selected
            # list from the for loop above
            for others in range(number_of_structure):
                if selected != others:

                    # in the selected list, disconnect all the uav in others list
                    for uav in uavs_list[selected]:
                        for disconnecting_uav in uavs_list[others]:

                            # check if uav is connected to disconnecting_UAV
                            if uav.checkUAVConnection(disconnecting_uav):
                                self.__CON.removeJoint(uav, disconnecting_uav)

                                disconnecting_uav.removeConnection(uav)
                                uav.removeConnection(disconnecting_uav)

        # create new structure
        del self.__structure[:]

        for groups in uavs_list:
            structure = Structure(groups[0])

            for i in range(1, len(group)):
                structure.connectSingleUAV(groups[i])

            self.__structure.append(structure)


    def structureGoToPose(self, structure, pose):
        pass
