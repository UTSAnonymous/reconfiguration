#!/usr/bin/env python
import time
import roslib
import rospy
import actionlib
import math

from geometry_msgs.msg import *
from hector_uav_msgs.msg import *

waypoints = []
current_goal = PoseStamped()
wp_threshold_high = 0.03
wp_threshold_low = -0.03
wp_counter = 0
quit_loop = False

#create callback for tf
def tf_callback(current_pos):
    global current_goal, wp_threshold_high, wp_threshold_low, wp_counter, quit_loop

    #print info to terminal
    rospy.loginfo("Current position: [{}, {}, {}]".format(current_pos.pose.position.x, current_pos.pose.position.y, current_pos.pose.position.z,))
    rospy.loginfo("Current Goal: [{}, {}, {}]".format(current_goal.pose.position.x, current_goal.pose.position.y, current_goal.pose.position.z,))
    #abs = current_pos.pose.position.y - current_goal.pose.position.y
    #rospy.loginfo("abs( current_pose - current_goal ): {}".format(abs))


#create function for generating waypoint
def generate_waypoints():
    #global var
    global waypoints

    #first waypoints
    goal_pose_1 = Pose()
    goal_pose_1.position.z = 5.0
    goal_pose_1.orientation.w = 1.0
    waypoints.append(goal_pose_1)

    #second waypoint
    goal_pose_2 = Pose()
    goal_pose_2.position.z = 5.0
    goal_pose_2.position.x = 1.0
    waypoints.append(goal_pose_2)

    #third waypoints
    goal_pose_3 = Pose()
    goal_pose_3.position.z = 5.0
    goal_pose_3.position.y = 1.0
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

#main for running the main loop
def main():
    global current_goal, quit_loop, wp_counter

    rospy.init_node("basic_waypoint", anonymous=True, disable_signals=True)
    rate = rospy.Rate(10)

    #local variables
    HBII0 = PoseStamped() #Initial pose in inertial coords. Set to zero.
    land_Goal = LandingGoal()
    pose_Goal = PoseGoal()
    takeoff_Goal = TakeoffGoal()

    #publisher and subscriber
    pos_sub = rospy.Subscriber('uav1/ground_truth_to_tf/pose',PoseStamped,tf_callback)

    #generate waypoint
    generate_waypoints()

    #format output message for actionlib
    current_goal.header.frame_id = 'uav1/world'
    current_goal.pose = waypoints[0]     #initialize first goal pose

    #show that node is running
    rospy.loginfo("Publishing position goal... \n")

    # initialise Action Client (Takeoff, Landing and Pose)
    posClient = actionlib.SimpleActionClient('uav1/action/pose',PoseAction)
    posClient.wait_for_server()
    rospy.loginfo("Pose client initialised.")

    takeoffClient = actionlib.SimpleActionClient('uav1/action/takeoff',TakeoffAction)
    takeoffClient.wait_for_server()
    rospy.loginfo("Takeoff client initialised.")

    landingClient = actionlib.SimpleActionClient('uav1/action/landing',LandingAction)
    landingClient.wait_for_server()
    rospy.loginfo("Landing client initialised.")

    #send takeoff goal (the action lib receive an empty input of
    #hector_uav_msgs -> TakeoffGoal msgs)
    takeoffClient.send_goal(TakeoffGoal)

    #main while loop
    while not rospy.is_shutdown() and ~quit_loop:
        current_goal.header.stamp = rospy.Time.now()
        ++current_goal.header.seq
        current_goal.header.frame_id = "uav1/world"

        pose_Goal.target_pose = current_goal
        posClient.send_goal(pose_Goal)
        success = posClient.wait_for_result(rospy.Duration(10))
        rospy.loginfo("loop")
        rospy.loginfo(success)
        wp_counter += 1
        if wp_counter < len(waypoints):
            current_goal.pose = waypoints[wp_counter]
            rospy.loginfo(wp_counter)
            rate.sleep()
        else:
            break

    #land the drone
    HBII0.header.stamp = rospy.Time.now()
    ++HBII0.header.seq
    HBII0.header.frame_id = "uav1/world"

    land_Goal.landing_zone = HBII0

    landingClient.send_goal(land_Goal)

    rospy.signal_shutdown("End")


if __name__ == '__main__': main()
