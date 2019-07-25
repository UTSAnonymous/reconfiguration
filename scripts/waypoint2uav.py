#!/usr/bin/env python
import time
import roslib
import rospy
import actionlib
import math
from UAV import UAV
from connection import Connection
from Structure import Structure

from geometry_msgs.msg import *
from hector_uav_msgs.msg import *

#create function for generating waypoint
def generate_waypoints_1():
    waypoints = []

    #first waypoints
    goal_pose_1 = Pose()
    goal_pose_1.position.x = 0.0
    goal_pose_1.position.y = 0.0
    goal_pose_1.position.z = 5.0
    goal_pose_1.orientation.w = 1.0
#    goal_pose_1.orientation.w = 0.7071068
#    goal_pose_1.orientation.z = 0.7071068
    waypoints.append(goal_pose_1)

    #second waypoint
    goal_pose_2 = Pose()
    goal_pose_2.position.x = 0.0
    goal_pose_2.position.y = 1.0
    goal_pose_2.position.z = 5.0
    goal_pose_2.orientation.w = 1.0
    #goal_pose_2.orientation.w = 0.7071068
    #goal_pose_2.orientation.z = 0.7071068
    waypoints.append(goal_pose_2)

    #third waypoints
    goal_pose_3 = Pose()
    goal_pose_3.position.x = 1.0
    goal_pose_3.position.y = 1.0
    goal_pose_3.position.z = 5.0
    goal_pose_3.orientation.w = 1.0
    waypoints.append(goal_pose_3)

    #fourth waypoints
    goal_pose_4 = Pose()
    goal_pose_4.position.x = 1.0
    goal_pose_4.position.y = 0.0
    goal_pose_4.position.z = 5.0
    goal_pose_4.orientation.w = 1.0
    waypoints.append(goal_pose_4)

    #fifth waypoint
    goal_pose_5 = Pose()
    goal_pose_5.position.z = 5.0
    goal_pose_5.position.x = -1.0
    goal_pose_5.position.y = -1.0
    waypoints.append(goal_pose_5)

    return waypoints

def generate_waypoints_2():
    waypoints = []

    #first waypoints
    goal_pose_1 = Pose()
    goal_pose_1.position.x = 0.0
    goal_pose_1.position.y = 1.0
    goal_pose_1.position.z = 5.0
    goal_pose_1.orientation.w = 1.0
#    goal_pose_1.orientation.w = 0.7071068
#    goal_pose_1.orientation.z = 0.7071068
    waypoints.append(goal_pose_1)

    #second waypoint
    goal_pose_2 = Pose()
    goal_pose_2.position.x = 0.0
    goal_pose_2.position.y = 2.0
    goal_pose_2.position.z = 5.0
    goal_pose_2.orientation.w = 1.0
    #goal_pose_2.orientation.w = 0.7071068
    #goal_pose_2.orientation.z = 0.7071068
    waypoints.append(goal_pose_2)

    #third waypoints
    goal_pose_3 = Pose()
    goal_pose_3.position.x = 1.0
    goal_pose_3.position.y = 2.0
    goal_pose_3.position.z = 5.0
    goal_pose_3.orientation.w = 1.0
    waypoints.append(goal_pose_3)

    #fourth waypoints
    goal_pose_4 = Pose()
    goal_pose_4.position.x = 1.0
    goal_pose_4.position.y = 1.0
    goal_pose_4.position.z = 5.0
    goal_pose_4.orientation.w = 1.0
    waypoints.append(goal_pose_4)

    #fifth waypoint
    goal_pose_5 = Pose()
    goal_pose_5.position.z = 5.0
    goal_pose_5.position.x = -1.0
    goal_pose_5.position.y = 0.0
    waypoints.append(goal_pose_5)

    return waypoints

#main for running the main loop
def main():
    wp_counter = 0

    rospy.init_node("basic_waypoint", anonymous=True, disable_signals=True)
    rate = rospy.Rate(10)

    waypoint_1 = generate_waypoints_1()
    waypoint_2 = generate_waypoints_2()

    #show that node is running
    rospy.loginfo("waypoint node running... \n")

    #create to object of UAV class
    UAV1 = UAV("uav1","base_link")
    UAV2 = UAV('uav2',"base_link")
    UAV3 = UAV('uav3',"base_link")
    UAV4 = UAV('uav4',"base_link")

    #takeoff

    UAV1.TakeoffAction()
    UAV2.TakeoffAction()
    UAV3.TakeoffAction()
    UAV4.TakeoffAction()

    #fly to Connection position
    UAV1.PoseAction(waypoint_1[0])
    time.sleep(2)
    UAV2.PoseAction(waypoint_1[1])
    time.sleep(2)
    UAV3.PoseAction(waypoint_1[2])
    time.sleep(2)
    UAV4.PoseAction(waypoint_1[3])
    UAV1.PoseActionWaitClient(50)
    UAV2.PoseActionWaitClient(50)
    UAV3.PoseActionWaitClient(50)
    UAV4.PoseActionWaitClient(50)

    time.sleep(7)

    # testing structure class
    MAIN_STRUCTURE = Structure(UAV1)

    MAIN_STRUCTURE.ConnectSingleUAV(UAV2)
    MAIN_STRUCTURE.ConnectSingleUAV(UAV3)
    MAIN_STRUCTURE.ConnectSingleUAV(UAV4)

    goal_pose = Pose()
    goal_pose.position.z = 5.0
    goal_pose.position.x = 0.0
    goal_pose.position.y = 0.0

    MAIN_STRUCTURE.StructureGoToPose(goal_pose)

    time.sleep(7)
    MAIN_STRUCTURE.DisconnectSingleUAV(UAV4)

    UAV4.PoseAction(waypoint_1[3])
    UAV4.PoseActionWaitClient(50)

    rospy.loginfo(str(len(UAV4.GetUAVConnection())))

    time.sleep(5)
    goal_pose.position.z = 3.0
    goal_pose.position.x = 0.0
    goal_pose.position.y = 0.0
    MAIN_STRUCTURE.StructureGoToPose(goal_pose)

    time.sleep(5)
    goal_pose.position.z = 3.0
    goal_pose.position.x = 1.0
    goal_pose.position.y = 1.0
    MAIN_STRUCTURE.StructureGoToPose(goal_pose)

    '''
    UAV1.PoseAction(waypoint_2[0])
    UAV2.PoseAction(waypoint_2[1])
    UAV3.PoseAction(waypoint_2[2])
    UAV4.PoseAction(waypoint_2[3])
    UAV1.PoseActionWaitClient(50)
    UAV2.PoseActionWaitClient(50)
    UAV3.PoseActionWaitClient(50)
    UAV4.PoseActionWaitClient(50)
    '''
    '''
    time.sleep(1)
    CON.Attach(UAV1.model_name,UAV1.link_name,UAV2.model_name,UAV2.link_name,"fixed", [0.5, -0.5, 0, 0, 0, 0, 0])
    time.sleep(1)

    UAV1.PoseAction(waypoint_1[1])
    UAV1.PoseActionWaitClient(50)

    UAV1.PoseAction(waypoint_1[1])
    UAV2.PoseAction(waypoint_2[1])
    result3 = UAV1.PoseActionWaitClient(50)
    result4 = UAV2.PoseActionWaitClient(50)
    rospy.loginfo(result3)
    rospy.loginfo(result4)

    UAV1.PoseAction(waypoint_1[2])
    UAV2.PoseAction(waypoint_2[2])
    result5 = UAV1.PoseActionWaitClient(50)
    result6 = UAV2.PoseActionWaitClient(50)
    rospy.loginfo(result5)
    rospy.loginfo(result6)

    time.sleep(1)
    CON.Detach(UAV1.model_name,UAV1.link_name,UAV2.model_name,UAV2.link_name)
    time.sleep(1)

    #time.sleep(20)
    UAV1.LandingAction()
    UAV2.LandingAction()
    '''

    rospy.signal_shutdown("End")


if __name__ == '__main__': main()
