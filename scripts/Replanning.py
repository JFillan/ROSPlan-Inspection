#!/usr/bin/env python
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_dispatch_msgs.msg import *

# Replanning function
def replan():
    # Update the PDDL problem definition to reflect the new state of the system
    get_state = rospy.ServiceProxy('/kcl_rosplan/get_current_knowledge', GetAttributeService)
    resp = get_state("robot_at", "")

    # Exclude the failed action from the set of available actions
    update_pddl_problem(resp.attributes)

    # Call the planner to generate a new plan
    new_plan = call_planner()

    # Dispatch the new plan
    dispatch_plan(new_plan)

# Define a callback function to be called when an action status message is received
def action_status_callback(data):
    if data.status == action_dispatch.ACTION_DISPATCH_FAILED:
        replan()

# Initialize the ROS node
rospy.init_node('replanning_node')

# Subscribe to the action status topic
rospy.Subscriber('/kcl_rosplan/action_dispatch', ActionDispatch, action_status_callback)

# Spin to keep the node running
rospy.spin()
