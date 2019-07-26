#!/usr/bin/env python

import rospy
import tf
import math
import time
from UAV import UAV
from gazebo_ros_link_attacher.srv import *

class Connection:

    def __init__(self):
        # connect to attachment ServiceProxy
        # both uses the attach srv msg (might be confusing at first)
        self.__attachSrv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.__detactSrv = rospy.ServiceProxy('/link_attacher_node/detach', Detach)

        self.__attachSrv.wait_for_service()
        self.__detactSrv.wait_for_service()
        rospy.loginfo("Connection service initialised")

    # function for Gazebo service call
    def attachService(self,model_name_1, link_name_1, model_name_2,link_name_2,type, pose):
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        req.type = type
        req.pose = pose

        result = self.__attachSrv.call(req)
        return result

    #function for Gazebo service call
    def detachService(self,model_name_1, link_name_1, model_name_2, link_name_2):
        req = DetachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2

        result = self.__detactSrv.call(req)
        return result

    def removeJoint(self, UAV1, UAV2):
        self.detachService(UAV1.model_name,UAV1.link_name,UAV2.model_name,UAV2.link_name)

    def fixedJoint(self, UAV1, UAV2):
        self.attachService(UAV1.model_name,UAV1.link_name,UAV2.model_name,UAV2.link_name,"fixed", [0, 0, 0, 0, 0, 0, 0])

    def revoluteJoint(self, UAV1, UAV2, isClockwise):
        pose1 = UAV1.GetPose()
        pose2 = UAV2.GetPose()

        # get transfrom from UAV1 to UAV2
        b_uav1_a = tf.transformations.translation_matrix((pose1.position.x, pose1.position.y, pose1.position.z))
        uav1_qx = tf.transformations.quaternion_about_axis(pose1.orientation.x, (1,0,0))
        uav1_qy = tf.transformations.quaternion_about_axis(pose1.orientation.y, (0,1,0))
        uav1_qz = tf.transformations.quaternion_about_axis(pose1.orientation.z, (0,0,1))
        uav1_q = tf.transformations.quaternion_multiply(uav1_qx, uav1_qy)
        uav1_q = tf.transformations.quaternion_multiply(uav1_q, uav1_qz)
        uav1_Rq = tf.transformations.quaternion_matrix(uav1_q)
        b_uav1_T = tf.transformations.concatenate_matrices(uav1_Rq, b_uav1_a)

        b_uav2_a = tf.transformations.translation_matrix((pose2.position.x, pose2.position.y, pose2.position.z))
        uav2_qx = tf.transformations.quaternion_about_axis(pose2.orientation.x, (1,0,0))
        uav2_qy = tf.transformations.quaternion_about_axis(pose2.orientation.y, (0,1,0))
        uav2_qz = tf.transformations.quaternion_about_axis(pose2.orientation.z, (0,0,1))
        uav2_q = tf.transformations.quaternion_multiply(uav2_qx, uav2_qy)
        uav2_q = tf.transformations.quaternion_multiply(uav2_q, uav2_qz)
        uav2_Rq = tf.transformations.quaternion_matrix(uav2_q)
        b_uav2_T = tf.transformations.concatenate_matrices(uav2_Rq, b_uav2_a)

        uav2_b_T = tf.transformations.inverse_matrix(b_uav2_T)
        uav2_uav1_T = tf.transformations.concatenate_matrices(uav2_b_T, b_uav1_T)

        # this result identify if the UAV or connected along x axis or y axis
        transl = tf.transformations.translation_from_matrix(uav2_uav1_T)

        A = (0.5, 0.5)
        B = (-0.5, 0.5)
        C = (-0.5, -0.5)
        D = (0.5, -0.5)

        #check if its connected horizontally
        x = round(transl[0],1)
        y = round(transl[1],1)

        if isClockwise == True:
            # horizontally
            if x == 0.0:
                if y > 0.0:
                    joint_x = -0.5
                    joint_y = 0.5
                else:
                    joint_x = 0.5
                    joint_y = -0.5
                # vertically
            else:
                if x > 0.0:
                    joint_x = 0.5
                    joint_y = 0.5
                else:
                    joint_x = -0.5
                    joint_y = -0.5
        # anticlockwise
        else:
            # horizontally
            if x == 0.0:
                if y > 0.0:
                    joint_x = 0.5
                    joint_y = 0.5
                else:
                    joint_x = -0.5
                    joint_y = -0.5
                # vertically
            else:
                if x > 0.0:
                    joint_x = 0.5
                    joint_y = -0.5
                else:
                    joint_x = 0.5
                    joint_y = -0.5

        time.sleep(1)
        result = self.attachService(UAV1.model_name,UAV1.link_name,UAV2.model_name,UAV2.link_name,"revolute", [joint_x, joint_y, 0, 0, 0, 0, 0])
        time.sleep(1)

        return result
