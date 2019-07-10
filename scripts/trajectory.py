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
    waypoints.append(goal_pose_1)

    #second waypoint
    goal_pose_2 = Pose()
    goal_pose_2.position.y = 0.0
    goal_pose_2.position.z = 5.0
    goal_pose_2.position.x = 1.0
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

def goal_to_pose(x, y, z, yaw):
    goal = Pose()

    goal.position.x = x
    goal.position.y = y
    goal.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    print quaternion
    goal.orientation.x = quaternion[0]
    goal.orientation.y = quaternion[1]
    goal.orientation.z = quaternion[2]
    goal.orientation.w = quaternion[3]
    return goal

#main for running the main loop
def main():
    wp_counter = 0

    rospy.init_node("basic_waypoint", anonymous=True, disable_signals=True)
    rate = rospy.Rate(10)

    waypoint_1 = generate_waypoints_1()

    #show that node is running
    rospy.loginfo("waypoint node running... \n")

    #create to object of UAV class
    UAV1 = UAV("uav1","base_link")
    CON = Connection()

    time.sleep(2)
    #takeoff

    UAV1.TakeoffAction()

    #fly to Connection position
    UAV1.PoseAction(waypoint_1[0])
    result1 = UAV1.PoseActionWaitClient(50)
    rospy.loginfo(result1)

    # Time counter
    t = 1.
    s = 100.
    n = 3
    # Circle loop
    while not rospy.is_shutdown():
        for i in range(n):
            if i == 1:
                theta = t / s + i * 2 * math.pi / n
                UAV1.PoseAction(goal_to_pose(math.cos(theta), math.sin(theta), 0.2*math.sin(1*theta)+5, theta + math.pi/2))

        t += 1
        rospy.sleep(.1)



    rospy.signal_shutdown("End")


if __name__ == '__main__': main()
