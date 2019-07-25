#!/usr/bin/env python

import time
import roslib
import rospy
import actionlib
import math
import threading

from geometry_msgs.msg import *
from hector_uav_msgs.msg import *

class UAV:

    def __init__(self, name, link_name):
        # name is important and it should be similar to the
        # groupns in the launch file
        self.__name = name
        self.model_name = name
        self.link_name = link_name

        self.__posClient = actionlib.SimpleActionClient(self.__name+'/action/pose',PoseAction)
        self.__takeoffClient = actionlib.SimpleActionClient(self.__name+'/action/takeoff',TakeoffAction)
        self.__landingClient = actionlib.SimpleActionClient(self.__name+'/action/landing',LandingAction)

        self.__posClient.wait_for_server()
        self.__takeoffClient.wait_for_server()
        self.__takeoffClient.wait_for_server()
        rospy.loginfo(self.__name+" actionlib initialised.")

        #subscriber
        self.__poseSub = rospy.Subscriber(self.__name+'/ground_truth_to_tf/pose',PoseStamped,self.tf_callback)

        #publisher
        self.__cmdVelPub = rospy.Publisher(self.__name+'/cmd_vel',Twist,queue_size=10)

        # rosmsg for pose action lib
        self.__PoseStamped = PoseStamped()
        self.__PoseGoal = PoseGoal()
        self.__called = False

        # rosmsg for takeoff and landing action
        self.__takeoffGoal = TakeoffGoal()
        self.__landingPose = PoseStamped()
        self.__landingGoal = LandingGoal() #TODO: add pose for takeoff and landing

        # array for UAV current pos
        self.__uavPoses = []

        # lock to prevent race condition in reading pose
        self.__lock = threading.Lock()

        # dict to store connection relation
        self.__connection = {}

    # get connection list
    def GetUAVConnection(self):
        return self.__connection

    # check connection list
    def CheckUAVConnection(self, UAV):
        if len(self.__connection) > 0:
            if UAV in self.__connection:
                return True
            else:
                return False
        else:
            return False

    # function for Adding a new UAV connection to this UAV
    def AddConnection(self, UAV, transform):
        # check if the UAV is connected to 4 drones already
        if len(self.__connection) < 4:
            self.__connection[UAV] = transform
            rospy.loginfo("Added UAV connection dict")
            return True
        else:
            return False

    # function for removing a UAV from the connection dict
    def RemoveConnection(self, UAV):

        if len(self.__connection) > 0:
            if UAV in self.__connection:
                del self.__connection[UAV]
                rospy.loginfo("Removed UAV from connection dict")
                return True
        else:
            return False

    def tf_callback(self, current_pos):
        # DEBUG: rospy.loginfo("Current position: [{}, {}, {}]".format(current_pos.pose.position.x, current_pos.pose.position.y, current_pos.pose.position.z,))
        #requires mutex/ lock to prevent race conditions
        self.__lock.acquire()
        self.__uavPoses.append(current_pos.pose)

        #check if length is more than 20, then delete the oldest one
        if len(self.__uavPoses) > 20:
            self.__uavPoses.pop(0)

        # DEBUG: rospy.loginfo(str(len(self.__uavPoses))+"    " + self.__name)
        self.__lock.release()

    def GetPose(self):
        pose = Pose()

        self.__lock.acquire()
        idx = len(self.__uavPoses) - 1

        # to prevent access to an empty list
        if idx != 0:
            pose = self.__uavPoses[idx]
        self.__lock.release()

        return pose

    def PoseAction(self, Pose):
        self.__PoseStamped.header.stamp = rospy.Time.now()
        ++self.__PoseStamped.header.seq
        self.__PoseStamped.header.frame_id = self.__name+"/world"
        self.__PoseStamped.pose = Pose

        self.__PoseGoal.target_pose = self.__PoseStamped
        self.__posClient.send_goal(self.__PoseGoal)
        self.__called = True

    def PoseActionWaitClient(self, Dur):
        if self.__called == True:
            self.__called = False
            result = self.__posClient.wait_for_result(rospy.Duration(Dur))

            return result

    def Hover(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0.00000000000001
        vel_msg.angular.z = 0
        self.__cmdVelPub.publish(vel_msg)

        vel_msg.angular.y = 0
        self.__cmdVelPub.publish(vel_msg)

    #TODO: fix takeoff, doesnt really work right now
    def TakeoffAction(self):
        #self.__takeoffClient.send_goal(TakeoffGoal())
        pass

    def LandingAction(self):
        self.__landingPose.header.stamp = rospy.Time.now()
        ++self.__landingPose.header.seq
        self.__landingPose.header.frame_id = self.__name+"/world"

        self.__landingGoal.landing_zone = self.__landingPose
        self.__landingClient.send_goal(self.__landingGoal)
        result = self.__landingClient.wait_for_result(rospy.Duration(5))
        return result
