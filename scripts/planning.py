#!/usr/bin/env python
import rospy
import os
import rosparam
import tf2_ros
import tf2_geometry_msgs
import time
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import Int32
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_dispatch_msgs.msg import ActionFeedback, ActionDispatch, GoalFailureCount, GoalFailureCountArray
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import KeyValue
from rospkg import RosPack
from geometry_msgs.msg import PoseStamped
from rosplan_interface_mapping.srv import AddWaypoint

dispatched_actions = {}
goal_failure_count = {}

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
    req.update_type.append(0)
    req.knowledge.append(KnowledgeItem(knowledge_type=1, instance_type='', instance_name='', attribute_name='robot_at', values=[KeyValue("v", "turtlebot"), KeyValue("wp", "wp0")], function_value=0.0))
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
        if not check_robot_at("/home/jonas/catkin_ws/src/inspection/common/problem2.pddl"):
            continue
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
            time.sleep(5)
            
    
    rospy.loginfo("Goals achived! Dispatch complete")
    rospy.signal_shutdown("Intended work completed, shutting down the node")
        
def cancel_dispatch(data):
    global goal_failure_count, dispatched_actions
    if data.status == 10:
        rospy.logerr("Action failed, canceling dispatch")

        # Get the action details from the dispatched_actions dictionary
        action_details = dispatched_actions.get(data.action_id)
        if action_details is None:
            rospy.logwarn("Failed to find action details for action_id: {}".format(data.action_id))
            return

        # Update the goal_failure_count dictionary
        goal_name = None
        action_params = {param.key: param.value for param in action_details.parameters}

        if action_details.name == "goto_waypoint":
            wp = action_params.get("to")
            goal_name = ("photographed", wp)
        elif action_details.name == "inspect":
            wp = action_params.get("wp")
            goal_name = ("photographed", wp)

        if goal_name is not None:
            if goal_name in goal_failure_count:
                goal_failure_count[goal_name] += 1
            else:
                goal_failure_count[goal_name] = 1
            print("Goal has failed " + str(goal_failure_count[goal_name]) + " times")
            # If the goal has failed 3 times, remove it from the knowledge base
            goal_failure_count_array = GoalFailureCountArray()
            goal_failure_count_array.goal_failure_count_array = [GoalFailureCount(goal_predicate=predicate, waypoint=int(wp[2:]), fail_count=count) for (predicate, wp), count in goal_failure_count.items()]
            goal_failure_count_pub.publish(goal_failure_count_array)
            if goal_failure_count[goal_name] >= 3:
                rospy.logwarn("Goal '{}' has failed 3 times. Removing it from the knowledge base.".format(goal_name))
                remove_goal(goal_name)
        
        cancel_dispatch_service = rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch', Empty)
        cancel_dispatch_service()

def remove_goal(goal_name):
    goal_predicate, wp = goal_name
    values = [KeyValue("wp", wp)]
    kb_update_service = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    req = KnowledgeUpdateServiceRequest(update_type=3, knowledge=KnowledgeItem(knowledge_type=1, instance_type='', instance_name='', attribute_name=goal_predicate, values=values))
    kb_update_service(req)

def check_robot_at(filename):
    found = False
    with open(filename, 'r') as file:
        for line in file:
            if "robot_at turtlebot" in line:
                found = True
                break
    if not found:
        added_new_waypoint = False
        if not added_new_waypoint:
            rospy.loginfo("Robot does not know where it is! Adding a new waypoint at its current position.")
            waypoint_added = add_waypoint()
            added_new_waypoint = True
            return False
    else: return True

def add_waypoint():
    rospy.loginfo("Adding new waypoint at the robot's current position.")
    waypoint_index = len(rospy.get_param('/rosplan_demo_waypoints/wp')) 
    waypoint_name = "wp" + str(waypoint_index)
    
    rospy.wait_for_service('/rosplan_roadmap_server/add_waypoint')
    add_waypoint_service = rospy.ServiceProxy('/rosplan_roadmap_server/add_waypoint', AddWaypoint)

    # Get the robot's current position from the /odom topic
    robot_odometry = rospy.wait_for_message('/odom', Odometry)

    # Extract the position and orientation from the Odometry message
    position = robot_odometry.pose.pose.position
    orientation = robot_odometry.pose.pose.orientation

    # Create a PoseStamped message
    robot_pose = PoseStamped()
    robot_pose.header.frame_id = "odom"
    robot_pose.pose.position = position
    robot_pose.pose.orientation = orientation

    # Transform the robot's position from the odom frame to the map frame
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    try:
        transform = tf_buffer.lookup_transform("map", "odom", rospy.Time(0), rospy.Duration(1.0))
        robot_pose_transformed = tf2_geometry_msgs.do_transform_pose(robot_pose, transform)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to transform robot's position from /odom to map frame.")
        return False
    
    # Call the add_waypoint service
    resp = add_waypoint_service(id=waypoint_name, waypoint=robot_pose_transformed, connecting_distance=20.0)
    
    # Add the robot_at predicate with the new waypoint
    update_array_service = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
    req = KnowledgeUpdateServiceArrayRequest(update_type=[], knowledge=[])
    req.update_type.append(0)
    req.knowledge.append(KnowledgeItem(knowledge_type=1, instance_type='', instance_name='', attribute_name='robot_at', values=[KeyValue("v", "turtlebot"), KeyValue("wp", waypoint_name)], function_value=0.0))
    update_array_service(req)

def action_dispatch_callback(data):
    global dispatched_actions
    dispatched_actions[data.action_id] = data

def goal_failure_count_callback(data):
    global goal_failure_count
    goal_failure_count = {(gfc.goal_predicate, "wp" + str(gfc.waypoint)): gfc.fail_count for gfc in data.goal_failure_count_array}

def main():
    global goal_failure_count_pub
    goal_failure_count_pub = rospy.Publisher('goal_failure_count', GoalFailureCountArray, queue_size=10)
    goal_failure_count_array = GoalFailureCountArray()
    goal_failure_count_array.goal_failure_count_array = list(goal_failure_count.values())
    goal_failure_count_pub.publish(goal_failure_count_array)
    while not rospy.is_shutdown():
        rospy.Subscriber("rosplan_plan_dispatcher/action_feedback", ActionFeedback, cancel_dispatch)
        rospy.Subscriber("rosplan_plan_dispatcher/action_dispatch", ActionDispatch, action_dispatch_callback)
        rospy.Subscriber('goal_failure_count', GoalFailureCountArray, goal_failure_count_callback)
        planning_sequence()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('planning_node')
    load_waypoints()
    add_initial_state_and_goals()
    main()