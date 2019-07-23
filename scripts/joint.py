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

#create function for generating waypoint
def generate_waypoints_1():
    waypoints = []

    #first waypoints
    goal_pose_1 = Pose()
    goal_pose_1.position.y = 0.0
    goal_pose_1.position.z = 5.0
    goal_pose_1.orientation.w = 1.0
#    goal_pose_1.orientation.w = 0.7071068
#    goal_pose_1.orientation.z = 0.7071068
    waypoints.append(goal_pose_1)

    #second waypoint
    goal_pose_2 = Pose()
    goal_pose_2.position.y = 0.0
    goal_pose_2.position.z = 5.0
    goal_pose_2.position.x = 1.0
    #goal_pose_2.orientation.w = 0.7071068
    #goal_pose_2.orientation.z = 0.7071068
    waypoints.append(goal_pose_2)

    #third waypoints
    goal_pose_3 = Pose()
    goal_pose_3.position.y = 1.0
    goal_pose_3.position.z = 5.0
    goal_pose_3.position.x = 5.0
    waypoints.append(goal_pose_3)

    #fourth waypoints
    goal_pose_4 = Pose()
    goal_pose_4.position.z = 5.0
    goal_pose_4.position.x = -1.0
    goal_pose_4.position.y = -2.0
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
    goal_pose_1.position.y = 1.0
    goal_pose_1.position.z = 5.0
    goal_pose_1.orientation.w = 1.0
    #goal_pose_1.orientation.w = 0.7071068
    #goal_pose_1.orientation.z = 0.7071068
    waypoints.append(goal_pose_1)

    #second waypoint
    goal_pose_2 = Pose()
    goal_pose_2.position.y = 1.0
    goal_pose_2.position.z = 5.0
    goal_pose_2.position.x = 0.0
    goal_pose_2.orientation.w = 0.7071068
    goal_pose_2.orientation.z = 0.7071068
    waypoints.append(goal_pose_2)

    #third waypoints
    goal_pose_3 = Pose()
    goal_pose_3.position.y = 2.0
    goal_pose_3.position.z = 5.0
    goal_pose_3.position.x = 5.0
    waypoints.append(goal_pose_3)

    #fourth waypoints
    goal_pose_4 = Pose()
    goal_pose_4.position.z = 5.0
    goal_pose_4.position.x = -1.0
    goal_pose_4.position.y = -1.0
    waypoints.append(goal_pose_4)

    #fifth waypoint
    goal_pose_5 = Pose()
    goal_pose_5.position.z = 5.0
    goal_pose_5.position.x = -1.0
    goal_pose_5.position.y = 0.0
    waypoints.append(goal_pose_5)

    return waypoints

def generate_waypoints_3():
    waypoints = []

    #first waypoints
    goal_pose_1 = Pose()
    goal_pose_1.position.y = 0.0
    goal_pose_1.position.z = 5.0
    goal_pose_1.orientation.w = 1.0
    waypoints.append(goal_pose_1)

    goal_pose_2 = Pose()
    goal_pose_2.position.y = 1.0
    goal_pose_2.position.z = 5.0
    goal_pose_2.orientation.w = 1.0
    waypoints.append(goal_pose_2)

    goal_pose_3 = Pose()
    goal_pose_3.position.y = 2.0
    goal_pose_3.position.z = 5.0
    goal_pose_3.orientation.w = 1.0
    waypoints.append(goal_pose_3)

    goal_pose_4 = Pose()
    goal_pose_4.position.y = 3.0
    goal_pose_4.position.z = 5.0
    goal_pose_4.orientation.w = 1.0
    waypoints.append(goal_pose_4)

    return waypoints


#main for running the main loop
def main():
    wp_counter = 0

    rospy.init_node("basic_waypoint", anonymous=True, disable_signals=True)
    rate = rospy.Rate(10)

    waypoint_1 = generate_waypoints_1()
    waypoint_2 = generate_waypoints_2()
    waypoint_3 = generate_waypoints_3()

    #show that node is running
    rospy.loginfo("waypoint node running... \n")

    #create to object of UAV class
    UAV1 = UAV("uav1","base_link")
    UAV2 = UAV('uav2',"base_link")
    UAV3 = UAV("uav3","base_link")
    UAV4 = UAV('uav4',"base_link")
    CON = Connection()

    time.sleep(2)
    #takeoff

    #fly to Connection position
    UAV1.PoseAction(waypoint_3[0])
    UAV2.PoseAction(waypoint_3[1])
    UAV3.PoseAction(waypoint_3[2])
    UAV4.PoseAction(waypoint_3[3])
    UAV1.PoseActionWaitClient(10)
    UAV2.PoseActionWaitClient(10)
    UAV3.PoseActionWaitClient(10)
    UAV4.PoseActionWaitClient(10)

    #code here go into a function
    #def rotate(parent, child(moving one))
    # UAV1 = parent(fixed), UAV2 = child. Child is the one that is rotating
    # ----------------- CODE ---------------- #
    pose1 = UAV1.GetPose()
    pose2 = UAV2.GetPose()
    clockwise = True

    CON.Attach(UAV1.model_name,UAV1.link_name,UAV2.model_name,UAV2.link_name,"fixed", [0, 0, 0, 0, 0, 0, 0])
    CON.Attach(UAV2.model_name,UAV2.link_name,UAV3.model_name,UAV3.link_name,"fixed", [0, 0, 0, 0, 0, 0, 0])
    CON.Attach(UAV3.model_name,UAV3.link_name,UAV4.model_name,UAV4.link_name,"fixed", [0, 0, 0, 0, 0, 0, 0])

    #-----------------------------------------#
    #time.sleep(1)
    #CON.Detach(UAV1.model_name,UAV1.link_name,UAV2.model_name,UAV2.link_name)
    #time.sleep(1)

    #UAV1.PoseAction(waypoint_1[1])
    #UAV1.PoseActionWaitClient(50)

    rospy.signal_shutdown("End")


if __name__ == '__main__': main()
