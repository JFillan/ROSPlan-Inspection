#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def send_to_wp0():
    # Initialize the ROS node
    rospy.init_node('send_to_wp0')

    # Create an action client to communicate with the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to become available
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    # Define the wp0 coordinates and orientation
    wp0 = PoseStamped()
    wp0.header.frame_id = "map"
    wp0.pose.position.x = 1.0  # Replace with the correct x coordinate
    wp0.pose.position.y = 1.0  # Replace with the correct y coordinate
    wp0.pose.orientation.w = 1.0  # Replace with the correct orientation

    # Create a MoveBaseGoal
    goal = MoveBaseGoal()
    goal.target_pose = wp0

    # Send the goal to the action server
    rospy.loginfo("Sending goal to move_base...")
    client.send_goal(goal)

    # Wait for the result
    rospy.loginfo("Waiting for the result...")
    client.wait_for_result()

    # Check the result and print a message
    result = client.get_state()
    if result == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot reached wp0.")
    else:
        rospy.logwarn("The robot failed to reach wp0.")

if __name__ == '__main__':
    try:
        send_to_wp0()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS node was interrupted.")
