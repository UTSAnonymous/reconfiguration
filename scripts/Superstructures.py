#!/usr/bin/env python
import time
import rospy
import roslib
import math
import collections
from tf.transformations import *
from Structure import Structure
from UAV import UAV
from connection import Connection

from geometry_msgs.msg import *
from hector_uav_msgs.msg import *

'''
1. Create a structure class which takes an input of UAV list/ number of drones
2. In the constructure then (maybe) create individual UAV object and insert into a Structure
- The structures class consist of 1 or multiple structure class
3. Consist of function which breaks or connect individual/ or multiple UAV
'''

class Superstructures():

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
            if i%4== 0 and i != 0:
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


    def connectStructure(self, list_1_name, list_2_name):
        '''
        1. check if the two list exist in the Superstructures
        2. loop thru each of them on the second list to determine their distance
        3. if less than 1.2m, create a connection
        4. append list_2 uav into list_1
        5. delete list_2 in self.__structure
        '''
        # loop thru the structure list
        check_1 = check_2 = False
        for structure in self.__structure:
            if collections.Counter(structure.getUAVNameList()) == collections.Counter(list_1_name):
                check_1 = True
                main_structure = structure
            if collections.Counter(structure.getUAVNameList()) == collections.Counter(list_2_name):
                check_2 = True
                secondary_structure = structure

        if check_1 == False and check_2 == False:
            rospy.loginfo("Connection list doesn't exist in the Superstructures class")
            return False

        #2 and 3. check distance of uav in list_2 with each uav in list_1
        secondary_uav_list = secondary_structure.getUAVList()

        for secondary in secondary_uav_list:
            main_structure.connectSingleUAV(secondary)

        #4 append those that are not connecting into a single structure
        main_uav_list = main_structure.getUAVList()

        for secondary in secondary_uav_list:
            if secondary not in main_uav_list:
                main_structure.appendUAVWithoutConnection(secondary)

        #5. delete list 2
        self.__structure.remove(secondary_structure)



    def disconnectStructure(self, uavs_name_list):
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


    def structureGoToPose(self, uav_name_list, pose):

        exist = False
        selectedStructure = None
        # loop thru the structure list
        for structure in self.__structure:
            if collections.Counter(structure.getUAVNameList()) == collections.Counter(uav_name_list):
                exist = True
                selectedStructure = structure

        if exist == False:
            rospy.loginfo("Structure with list of uav does not exist")
            return False
        else:
            selectedStructure.structurePoseAction(pose)

    def getConnectionDict(self):
        '''
        1. check if there is only one structure in this class
        2. get all of the UAV in the structure
        3. do similar breath first search algorithm
        4. convert the 2d coordinate to 2d list/ np array
        '''

        # check if there is only one structure
        if len(self.__structure) != 1:
            rospy.loginfo("Too many structure in this structures.")
            return False

        # get all the uav in the structure
        uavs = self.__structure[0].getUAVList()
        queue = []
        completed = {}

        # put the first uav into the queue and completed
        queue.append(uavs[0])
        counter = 0

        # first UAV has the coordiante (0,0)
        # coordinates is in a list
        completed[uavs[0]] = [0,0]

        while True:
            current_uav = queue[counter]
            connected_uav_dict = current_uav.getUAVConnection()

            # loop through the list and add it to the queue if doesn'
            # exist in the queue
            for uav, trans in connected_uav_dict.items():
                if uav not in queue:

                    sc, sh, a, transl, p = decompose_matrix(trans)
                    x = y = 0

                    # check if connected in x or y direction
                    # if x val > y val
                    if abs(transl[0]) > abs(transl[1]):
                        if transl[0] > 0:
                            x = 1
                        else:
                            x = -1
                    else:
                        if transl[1] > 0:
                            y = 1
                        else:
                            y = -1

                    current_pos = completed[current_uav]
                    new_pos = [current_pos[0]+x, current_pos[1]+y]
                    completed[uav] = new_pos

                    queue.append(uav)

            if len(completed) == len(uavs):
                break

            counter += 1

        # convert 2d coordinate to start from (0,0)
        smallest_x = smallest_y = 0
        for uav, coordinate in completed.items():

            # get the smallest value in x and y
            if coordinate[0] < smallest_x:
                smallest_x = coordinate[0]
            if coordinate[1] < smallest_y:
                smallest_x = coordinate[1]

        # shift the smallest x and y = 0
        for uav, coordinate in completed.items():

            coordinate[0] = coordinate[0] - smallest_x
            coordinate[1] = coordinate[1] - smallest_y

        # output dict with key = uav name
        output = {}
        for uav, coordinate in completed.items():
            output[uav.model_name] = coordinate

        return output

        '''
        # convert to 8x8 2d list
        output_array = [["0" for i in range(8)] for j in range(8)]

        for i in range(8):
            for j in range(8):

                target = [j,i]
                # loop thru completed to look for matching uav coor
                for uav, coordinate in completed.items():
                    if target == coordinate:
                        output_array[i][j] = uav.model_name

        return output_array
        '''

    def getStructureList(self):
        return self.__structure

    def temp(self):
        list = self.__structure[0].getUAVList()
        dict = list[0].getUAVConnection()

        for uav,tran in dict.items():
            print(uav.model_name)
