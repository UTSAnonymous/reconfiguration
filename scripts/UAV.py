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
