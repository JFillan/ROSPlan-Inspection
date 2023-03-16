#!/usr/bin/env python
import rospy
import os
import rosparam
import actionlib
import tf
from std_srvs.srv import Trigger, Empty
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_dispatch_msgs.msg import ActionFeedback
from diagnostic_msgs.msg import KeyValue
from rospkg import RosPack
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


def load_waypoints():
    # load waypoints into parameter server
    rospy.loginfo("Loading waypoints")
    package_path = RosPack().get_path('inspection')
    waypoints_path = os.path.join(package_path, 'config', 'waypoints.yaml')
    paramlist = rosparam.load_file(waypoints_path)
    for params, ns in paramlist:
        rosparam.upload_params(ns,params)
    # let RoadmapServer know that waypoints are available in param server
    rospy.wait_for_service('/rosplan_roadmap_server/load_waypoints')
    load_waypoints_service = rospy.ServiceProxy('/rosplan_roadmap_server/load_waypoints', Trigger)
    load_waypoints_service()

def add_initial_state_and_goals():
    rospy.loginfo("Adding initial state and goals to knowledge base.")
    update_array_service = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
    req = KnowledgeUpdateServiceArrayRequest(update_type=[], knowledge=[])
    req.update_type.append(0)
    req.knowledge.append(KnowledgeItem(knowledge_type=0, instance_type='robot', instance_name='turtlebot', attribute_name='', function_value=0.0))
    # req.update_type.append(0)
    # req.knowledge.append(KnowledgeItem(knowledge_type=1, instance_type='', instance_name='', attribute_name='robot_at', values=[KeyValue("v", "turtlebot"), KeyValue("wp", "wp0")], function_value=0.0))
    for i in range(0, 2):
        req.update_type.append(0)
        req.knowledge.append(KnowledgeItem(knowledge_type=1, instance_type='', instance_name='', attribute_name='charge_at', values=[KeyValue("wp", "wp" + str(i))], function_value=0.0))
    update_array_service(req)
    
def planning_sequence():
    goal_achieved = False
    while goal_achieved != True:
        # Automatically generate PDDL problem from KB snapshot (e.g. fetch knowledge from KB and create problem.pddl)
        rospy.loginfo("Calling problem generator.")
        planner_service = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
        planner_service()
        # Check that the robot knows where it is
        check_robot_at("/home/jonas/catkin_ws/src/inspection/common/problem2.pddl")
        # Make plan
        rospy.loginfo("Calling planner interface.")
        planner_service = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
        planner_service()
        # Parse plan 
        rospy.loginfo("Calling plan parser.")
        parsing_service = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
        parsing_service()
        # Dispatch (execute) plan.
        rospy.loginfo("Calling plan dispatcher.")
        dispatch_service = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
        resp = dispatch_service()
        goal_achieved = resp.goal_achieved
        if goal_achieved == False:
            rospy.logerr("Dispatch was canceled without all goals being achived. Replanning")
    
    rospy.loginfo("Goals achived! Dispatch complete")
    rospy.signal_shutdown("Intended work completed, shutting down the node")
        
def cancel_dispatch(data):
    if data.status == 10:
        rospy.logerr("Action failed, canceling dispatch")
        cancel_dispatch_service = rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch', Empty)
        cancel_dispatch_service()

def check_robot_at(filename):
    found = False
    with open(filename, 'r') as file:
        for line in file:
            if "robot_at turtlebot" in line:
                found = True
                break
    if not found:
        rospy.loginfo("Robot does not know where it is! Redirecting to wp0")
        goal_reached = False
        while not goal_reached:
            goal_reached = move_to_wp0()

def move_to_wp0():
    update_array_service = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
    req = KnowledgeUpdateServiceArrayRequest(update_type=[], knowledge=[])
    
    # Create an action client to communicate with the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to become available
    client.wait_for_server()

    # Get the wp0 coordinates and orientation from the parameter server
    wp = rospy.get_param('/rosplan_demo_waypoints/wp/wp0')
    # Define the PoseStamped message
    quaternion = tf.transformations.quaternion_from_euler(0, 0, wp[2])

    # Define the PoseStamped message
    wp_pose = PoseStamped()
    wp_pose.header.frame_id = "map"
    wp_pose.pose.position.x = wp[0]
    wp_pose.pose.position.y = wp[1]
    wp_pose.pose.orientation.x = quaternion[0]
    wp_pose.pose.orientation.y = quaternion[1]
    wp_pose.pose.orientation.z = quaternion[2]
    wp_pose.pose.orientation.w = quaternion[3]

    # Create a MoveBaseGoal
    goal = MoveBaseGoal()
    goal.target_pose = wp_pose

    # Send the goal to the action server
    client.send_goal(goal)
    # Wait for the result
    client.wait_for_result()

    # Check the result and print a message
    result = client.get_state()
    if result == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot reached wp0.")
        req.update_type.append(0)
        req.knowledge.append(KnowledgeItem(knowledge_type=1, instance_type='', instance_name='', attribute_name='robot_at', values=[KeyValue("v", "turtlebot"), KeyValue("wp", "wp0")], function_value=0.0))
        update_array_service(req)
        return True
    else:
        rospy.logwarn("The robot failed to reach wp0.")
        return False


def main():
    while not rospy.is_shutdown():
        rospy.Subscriber("rosplan_plan_dispatcher/action_feedback", ActionFeedback, cancel_dispatch)
        planning_sequence()
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('planning_node')
    load_waypoints()
    add_initial_state_and_goals()
    main()