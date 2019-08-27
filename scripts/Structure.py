#!/usr/bin/env python

import time
import roslib
import rospy
import actionlib
import math
from tf.transformations import *
from UAV import UAV
from connection import Connection

from geometry_msgs.msg import *
from hector_uav_msgs.msg import *

# for this structure class, we assume the UAV structure is 2D and all of the UAV
# are on the same plane
class Structure:

    def __init__(self, UAV):
        self.__UAVS = [UAV]
        self.__CON = Connection()
        self.__centroid = Pose()
        self.__tolerance = 0.05

    def connectSingleUAV(self, connecting_UAV):
        '''
        1. Check if the connecting UAV exist in the Structure
        2. Check which existing UAV in the structure is closest to it
        3. Create connection with all UAV perpendicular to it
        4. Add connection to each UAV connection dict
        '''

        if connecting_UAV not in self.__UAVS:
            poseUAV = connecting_UAV.getPose()
            closest_existing_UAV = []

            for existing_UAV in self.__UAVS:

                # calculate the distance between each of the existing_UAV with UAV
                pose = existing_UAV.getPose()
                dist = math.sqrt( (pose.position.x-poseUAV.position.x)**2 + (pose.position.y-poseUAV.position.y)**2 )

                # store UAV that is close to the connecting UAV in a list
                if dist < 1.2:
                    closest_existing_UAV.append(existing_UAV)

            # check if number of UAV connecting is more that 4
            if len(closest_existing_UAV) > 4:
                rospy.loginfo("Too many connection!")
                return False
            if len(closest_existing_UAV) == 0:
                return False
            else:
                rospy.loginfo("Number of connection created: " + str(len(closest_existing_UAV)))

            #TODO: check if any of the existing_UAV are connected to the connecting UAV already

            # Create connection with each of the UAV and add connection dict
            b_connecting_T = self.createTFMatrix(connecting_UAV.getPose())
            connecting_b_T = inverse_matrix(b_connecting_T)
            for UAV in closest_existing_UAV:

                rospy.loginfo("Connecting " + UAV.model_name + " with " + connecting_UAV.model_name)

                # calculate transfrom for each connection
                b_UAV_T = self.createTFMatrix(UAV.getPose())
                connecting_UAV_T = concatenate_matrices(connecting_b_T, b_UAV_T)
                UAV_connecting_T = inverse_matrix(connecting_UAV_T)

                self.__CON.fixedJoint(UAV, connecting_UAV)
                UAV.addConnection(connecting_UAV, UAV_connecting_T)
                connecting_UAV.addConnection(UAV, connecting_UAV_T)

            # add connecting UAV to structure
            self.__UAVS.append(connecting_UAV)

            return True

    def disconnectSingleUAV(self, disconnecting_UAV):

        # remove UAV from Structure
        self.__UAVS.remove(disconnecting_UAV)

        # return a dict
        connection = disconnecting_UAV.getUAVConnection()

        uav_list = []
        for UAV in connection:
            rospy.loginfo("Disconnecting " + UAV.model_name + " from " + disconnecting_UAV.model_name)
            self.__CON.removeJoint(UAV, disconnecting_UAV)
            uav_list.append(UAV)

        for UAV in uav_list:
            disconnecting_UAV.removeConnection(UAV)
            UAV.removeConnection(disconnecting_UAV)

    def getCentroid(self):
        xT = 0
        yT = 0
        zT = 0
        n = len(self.__UAVS)

        for UAV in self.__UAVS:
            pose = UAV.getPose()
            xT = xT + pose.position.x
            yT = yT + pose.position.y
            zT = zT + pose.position.z

        return (xT/n, yT/n, zT/n)

    #TODO: Add rotation into pose action
    def structurePoseAction(self, goalPose):
        '''
        1. Calculate the centroid of the Structure
        2. Calculate the relative transfrom from centroid to each of the UAV
        3. For the new pose, calculate the new pose for the individual UAV
        4. Send new pose commands to each UAV

        *Blocking function
        '''
        # calculate centroid of structure
        xC, yC, zC = self.getCentroid()

        centroidPoint = Point(float(xC),float(yC),float(zC))
        centroidQuaternion = Quaternion(0,0,0,1.0)
        centroidPose = Pose(centroidPoint, centroidQuaternion)

        b_centroid_T = self.createTFMatrix(centroidPose)
        centroid_b_T = inverse_matrix(b_centroid_T)

        # for the new pose, calculate new pose for individual UAV
        goal_T = self.createTFMatrix(goalPose)

        # calculate relative transform
        UAV_goal = []
        for UAV in self.__UAVS:
            pose = UAV.getPose()
            b_UAV_T = self.createTFMatrix(pose)

            centroid_UAV_T = concatenate_matrices(centroid_b_T, b_UAV_T)
            b_UAV_T_new = concatenate_matrices(goal_T, centroid_UAV_T)

            #convert tf to pose
            x, y, z = translation_from_matrix(b_UAV_T_new)
            point = Point(x, y, z)
            quaternion = Quaternion(0, 0, 0, 1.0)
            new_pose = Pose(point, quaternion)

            UAV_goal.append(new_pose)

        # interpolate goal for a smoother trajectory

        UAV_interpolated_goal = []
        steps = 50
        for i in range(1,steps+1):
            current_goal = []
            for j in range(len(UAV_goal)):
                pose = self.__UAVS[j].getPose()
                goal_pose = UAV_goal[j]

                #calculate interpolated goal
                if -self.__tolerance < goal_pose.position.x - pose.position.x < self.__tolerance:
                    x_new = goal_pose.position.x
                else:
                    x_new = (((goal_pose.position.x - pose.position.x)/steps) * i) + pose.position.x

                if -self.__tolerance < goal_pose.position.y - pose.position.y < self.__tolerance:
                    y_new = goal_pose.position.y
                else:
                    y_new = (((goal_pose.position.y - pose.position.y)/steps) * i) + pose.position.y

                if -self.__tolerance < goal_pose.position.z - pose.position.z < self.__tolerance:
                    z_new = goal_pose.position.z
                else:
                    z_new = (((goal_pose.position.z - pose.position.z)/steps) * i) + pose.position.z

                #DEBUG: print("x : " + str(x_new) + "y : " + str(y_new) + "z : " + str(z_new))

                point = Point(x_new, y_new, z_new)
                quaternion = Quaternion(goal_pose.orientation.x, goal_pose.orientation.y,
                                                goal_pose.orientation.z, goal_pose.orientation.w)
                new_pose = Pose(point, quaternion)

                current_goal.append(new_pose)

            UAV_interpolated_goal.append(current_goal)

        # send new pose commands
        for UAV_goal in UAV_interpolated_goal:
            for i in range(len(UAV_goal)):
                self.__UAVS[i].poseAction(UAV_goal[i])
            time.sleep(0.4)

        # wait for execution to end
        for i in range(len(self.__UAVS)):
            self.__UAVS[i].poseActionWaitClient(50)

        rospy.loginfo("UAV structure translation completed!")

        # calculate and execute rotation
        '''
        1. determine if there is a z_rotation (convert rotation in terms of 90 and 180)
        2. get centroid and calculate distance of each uav to centroid
        3. create a for loop that generate the path for each uav
        4. publish
        '''

        # determine z_rotation (goal_T, centroid_b_T, b_centroid_T)
        rotation = euler_from_matrix(goal_T,'rxyz')

        if rotation[2] == 0.0:
            rospy.loginfo("UAV structure movement done!")
            return True
        else:
            z_rot = int(round(rotation[2]/math.pi*180,-1))
            if z_rot > 270:
                z_rot = -90
            if z_rot < -270:
                z_rot = 90
            if z_rot == 360:
                rospy.loginfo("UAV structure movement done!")
                return True

            # get centroid and calculate distance to each uav
            uavs_radius = []

            # calculate centroid of structure
            xC, yC, zC = self.getCentroid()

            centroidPoint = Point(float(xC),float(yC),float(zC))
            centroidQuaternion = Quaternion(0,0,0,1.0)
            centroidPose = Pose(centroidPoint, centroidQuaternion)

            b_centroid_T = self.createTFMatrix(centroidPose)
            centroid_b_T = inverse_matrix(b_centroid_T)
            centroid_transl = translation_from_matrix(b_centroid_T)

            for uav in self.__UAVS:
                pose = uav.getPose()
                dist = math.sqrt( (pose.position.x-centroid_transl[0])**2 + (pose.position.y-centroid_transl[1])**2 )
                uavs_radius.append(dist)

            # generate a rotational trajectory with 1 degree interval
            uavs_goal = []

            for i in range(1,abs(z_rot)+1):
                if z_rot < 0:
                    angle = -i
                else:
                    angle = i

                goal = []
                for uav_element in range(len(self.__UAVS)):

                    x = (uavs_radius[uav_element] * math.sin(-angle)) + centroid_transl[0]
                    y = (uavs_radius[uav_element] * math.cos(angle)) + centroid_transl[1]
                    z = goalPose.position.z

                    point = Point(x,y,z)
                    quat = quaternion_about_axis(angle,(0,0,1))
                    new_pose = Pose(point, quaternion)

                    goal.append(new_pose)

                uavs_goal.append(goal)

            # send new pose commands
            for goals in uavs_goal:
                for i in range(len(goals)):
                    self.__UAVS[i].poseAction(goals[i])
                time.sleep(0.4)

            # wait for execution to end
            for i in range(len(self.__UAVS)):
                self.__UAVS[i].poseActionWaitClient(50)

            rospy.loginfo("UAV structure movement done!")


    def createTFMatrix(self, pose):

        b_uav1_T = translation_matrix((pose.position.x, pose.position.y, pose.position.z))
        quaternion = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        uav1_qx = quaternion_about_axis(euler[0], (1,0,0))
        uav1_qy = quaternion_about_axis(euler[1], (0,1,0))
        uav1_qz = quaternion_about_axis(euler[2], (0,0,1))
        uav1_q = quaternion_multiply(uav1_qx, uav1_qy)
        uav1_q = quaternion_multiply(uav1_q, uav1_qz)
        uav1_Rq = quaternion_matrix(uav1_q)
        b_uav1_T = concatenate_matrices(uav1_Rq, b_uav1_T)

        return b_uav1_T

    def appendUAVWithoutConnection(self, uav):
        self.__UAVS.append(uav)

    def getNumOfUAV(self):
        return len(self.__UAVS)

    def getUAVList(self):
        return self.__UAVS

    def getCentreOfMass(self):
        pass

    def getUAVNameList(self):
        name = []
        for uav in self.__UAVS:
            name.append(uav.model_name)

        return name
