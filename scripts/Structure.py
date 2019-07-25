#!/usr/bin/env python

import time
import roslib
import rospy
import actionlib
import math
import tf
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

    def ConnectSingleUAV(self, connecting_UAV):
        '''
        1. Check if the connecting UAV exist in the Structure
        2. Check which existing UAV in the structure is closest to it
        3. Create connection with all UAV perpendicular to it
        4. Add connection to each UAV connection dict
        '''

        if connecting_UAV not in self.__UAVS:
            poseUAV = connecting_UAV.GetPose()
            closest_existing_UAV = []

            for existing_UAV in self.__UAVS:

                # calculate the distance between each of the existing_UAV with UAV
                pose = existing_UAV.GetPose()
                dist = math.sqrt( (pose.position.x-poseUAV.position.x)**2 + (pose.position.y-poseUAV.position.y)**2 )

                # store UAV that is close to the connecting UAV in a list
                if dist < 1.2:
                    closest_existing_UAV.append(existing_UAV)

            # check if number of UAV connecting is more that 4
            if len(closest_existing_UAV) > 4:
                rospy.loginfo("Too many connection!")
                return False
            else:
                rospy.loginfo("Number of connection created: " + str(len(closest_existing_UAV)))

            #TODO: check if any of the existing_UAV are connected to the connecting UAV already

            # Create connection with each of the UAV and add connection dict
            b_connecting_T = self.CreateTFMatrix(connecting_UAV.GetPose())
            connecting_b_T = tf.transformations.inverse_matrix(b_connecting_T)
            for UAV in closest_existing_UAV:

                rospy.loginfo("Connecting " + UAV.model_name + " with " + connecting_UAV.model_name)

                # calculate transfrom for each connection
                b_UAV_T = self.CreateTFMatrix(UAV.GetPose())
                connecting_UAV_T = tf.transformations.concatenate_matrices(connecting_b_T, b_UAV_T)
                UAV_connecting_T = tf.transformations.inverse_matrix(connecting_UAV_T)

                self.__CON.FixedJoint(UAV, connecting_UAV)
                UAV.AddConnection(connecting_UAV, UAV_connecting_T)
                connecting_UAV.AddConnection(UAV, connecting_UAV_T)

            # add connecting UAV to structure
            self.__UAVS.append(connecting_UAV)

            return True

    def DisconnectSingleUAV(self, disconnecting_UAV):

        # remove UAV from Structure
        self.__UAVS.remove(disconnecting_UAV)

        # return a dict
        connection = disconnecting_UAV.GetUAVConnection()

        uav_list = []
        for UAV in connection:
            rospy.loginfo("Disconnecting " + UAV.model_name + " from " + disconnecting_UAV.model_name)
            self.__CON.RemoveJoint(UAV, disconnecting_UAV)
            uav_list.append(UAV)

        for UAV in uav_list:
            disconnecting_UAV.RemoveConnection(UAV)
            UAV.RemoveConnection(disconnecting_UAV)

    def GetCentroid(self):
        xT = 0
        yT = 0
        zT = 0
        n = len(self.__UAVS)

        for UAV in self.__UAVS:
            pose = UAV.GetPose()
            xT = xT + pose.position.x
            yT = yT + pose.position.y
            zT = zT + pose.position.z

        return (xT/n, yT/n, zT/n)

    def StructureGoToPose(self, goalPose):
        '''
        1. Calculate the centroid of the Structure
        2. Calculate the relative transfrom from centroid to each of the UAV
        3. For the new pose, calculate the new pose for the individual UAV
        4. Send new pose commands to each UAV

        *Blocking function
        '''
        # calculate centroid of structure
        xC, yC, zC = self.GetCentroid()

        centroidPoint = Point(float(xC),float(yC),float(zC))
        centroidQuaternion = Quaternion(0,0,0,1.0)
        centroidPose = Pose(centroidPoint, centroidQuaternion)

        b_centroid_T = self.CreateTFMatrix(centroidPose)
        centroid_b_T = tf.transformations.inverse_matrix(b_centroid_T)

        # for the new pose, calculate new pose for individual UAV
        goal_T = self.CreateTFMatrix(goalPose)

        # calculate relative transform
        UAV_goal = []
        for UAV in self.__UAVS:
            pose = UAV.GetPose()
            b_UAV_T = self.CreateTFMatrix(pose)

            centroid_UAV_T = tf.transformations.concatenate_matrices(centroid_b_T, b_UAV_T)
            b_UAV_T_new = tf.transformations.concatenate_matrices(goal_T, centroid_UAV_T)

            #convert tf to pose
            x, y, z = tf.transformations.translation_from_matrix(b_UAV_T_new)
            point = Point(x, y, z)
            quaternion = Quaternion(pose.orientation.x, pose.orientation.y,
                                            pose.orientation.z, pose.orientation.w)
            new_pose = Pose(point, quaternion)

            UAV_goal.append(new_pose)

        # send new pose commands
        for i in range(len(UAV_goal)):
            self.__UAVS[i].PoseAction(UAV_goal[i])

        # wait for execution to end
        for i in range(len(UAV_goal)):
            self.__UAVS[i].PoseActionWaitClient(50)


    def CreateTFMatrix(self, pose):

        b_uav1_T = tf.transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
        uav1_qx = tf.transformations.quaternion_about_axis(pose.orientation.x, (1,0,0))
        uav1_qy = tf.transformations.quaternion_about_axis(pose.orientation.y, (0,1,0))
        uav1_qz = tf.transformations.quaternion_about_axis(pose.orientation.z, (0,0,1))
        uav1_q = tf.transformations.quaternion_multiply(uav1_qx, uav1_qy)
        uav1_q = tf.transformations.quaternion_multiply(uav1_q, uav1_qz)
        uav1_Rq = tf.transformations.quaternion_matrix(uav1_q)
        b_uav1_T = tf.transformations.concatenate_matrices(uav1_Rq, b_uav1_T)

        return b_uav1_T

    def GetNumOfUAV(self):
        return len(self.__UAVS)

    def StrucPoseAction(self, Pose):
        pass

    def GetCentreOfMass(self):
        pass
