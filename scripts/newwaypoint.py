#!/usr/bin/env python
import time
import roslib
import rospy
import actionlib
import math
from posecommands import simplifiedPoseMsg
from UAV import UAV
from Structure import Structure
from Superstructures import Superstructures

from geometry_msgs.msg import *
from hector_uav_msgs.msg import *

#create function for generating waypoint
def generate_waypoints_1():
    waypoints = []

    #first waypoints
    goal_pose_1 = Pose()
    goal_pose_1 = simplifiedPoseMsg(0,0,5,90)
    waypoints.append(goal_pose_1)

    #second waypoint
    goal_pose_2 = Pose()
    goal_pose_2 = simplifiedPoseMsg(1.5,1,5,0)
    waypoints.append(goal_pose_2)

    #third waypoints
    goal_pose_3 = Pose()
    goal_pose_3 = simplifiedPoseMsg(-1,1,5,0)
    waypoints.append(goal_pose_3)

    #fourth waypoints
    goal_pose_4 = Pose()
    goal_pose_4 = simplifiedPoseMsg(0,0,5,90)
    waypoints.append(goal_pose_4)

    #fifth waypoint
    goal_pose_5 = Pose()
    goal_pose_5 = simplifiedPoseMsg(0,0,5,-90)
    waypoints.append(goal_pose_5)

    return waypoints

#main for running the main loop
def main():
    rospy.init_node("basic_waypoint", anonymous=True, disable_signals=True)
    rate = rospy.Rate(10)

    waypoint_1 = generate_waypoints_1()

    #show that node is running
    rospy.loginfo("waypoint node running... \n")

    # testing Structures
    main = Superstructures(4)
    group = ["uav1","uav2","uav3","uav4"]
    main.structureGoToPose(group,waypoint_1[0])

    group1 = ["uav1","uav2"]
    group2 = ["uav3","uav4"]
    list = [group1, group2]
    main.disconnectStructure(list)
    print("Disconnected list")
    list = main.getStructureList()
    print(list)
    print(len(list))


    #main.connectStructure(group1, group2)
    #print("Connected list")
    #print(main.getStructureList())

    rospy.signal_shutdown("End")


if __name__ == '__main__': main()
