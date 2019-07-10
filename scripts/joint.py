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
    CON = Connection()

    time.sleep(2)
    #takeoff

    UAV1.TakeoffAction()
    UAV2.TakeoffAction()

    #fly to Connection position
    UAV1.PoseAction(waypoint_1[0])
    UAV2.PoseAction(waypoint_2[0])
    result1 = UAV1.PoseActionWaitClient(10)
    result2 = UAV2.PoseActionWaitClient(10)
    rospy.loginfo(result1)
    rospy.loginfo(result2)

    #code here go into a function
    #def rotate(parent, child(moving one))
    # UAV1 = parent(fixed), UAV2 = child. Child is the one that is rotating
    # ----------------- CODE ---------------- #
    pose1 = UAV1.GetPose()
    pose2 = UAV2.GetPose()
    clockwise = True
    goal = Pose()
    goal.position.x = 1.0
    goal.position.y = 0.0
    goal.position.z = 5.0

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

    # rotation depends on clockwise or anticlockwise rotation
    if clockwise == True:
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
    CON.Attach(UAV1.model_name,UAV1.link_name,UAV2.model_name,UAV2.link_name,"revolute", [joint_x, joint_y, 0, 0, 0, 0, 0])
    time.sleep(1)

    #generate trajectory

    #execute trajectory

    
    #-----------------------------------------#
    time.sleep(1)
    CON.Detach(UAV1.model_name,UAV1.link_name,UAV2.model_name,UAV2.link_name)
    time.sleep(1)

    #UAV1.PoseAction(waypoint_1[1])
    #UAV1.PoseActionWaitClient(50)

    rospy.signal_shutdown("End")


if __name__ == '__main__': main()
