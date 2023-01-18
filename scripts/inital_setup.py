#!/usr/bin/env python
import rospy
import os
import rosparam
from std_srvs.srv import Trigger, Empty
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_dispatch_msgs.msg import ActionFeedback
from diagnostic_msgs.msg import KeyValue
from rospkg import RosPack


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
    for i in range(1, 3):
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

def main():
    while not rospy.is_shutdown():
        rospy.Subscriber("rosplan_plan_dispatcher/action_feedback", ActionFeedback, cancel_dispatch)
        planning_sequence()
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('your_node_name')
    load_waypoints()
    add_initial_state_and_goals()
    main()