#!/usr/bin/env python
import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback, GoalFailureCount, GoalFailureCountArray
from inputimeout import inputimeout, TimeoutOccurred
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
from std_srvs.srv import Empty


# Prompt the operator every time a photo is taken (inspect action).
# Operator rejects (pressing ENTER) or approves (waits for 10 sec).
# If rejected, "photographed" predicate is removed and dispatch is canceled, triggering replanning.
class InspectionManager:

    def __init__(self):
        self.active_action_id = None
        self.active_name = None
        self.active_wp = None
        self.should_cancel_dispatch = False
        self.goal_failure_count = {}

        rospy.init_node('inspect_subscriber')
        rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch', ActionDispatch, self.inspect_callback)
        rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback', ActionFeedback, self.status_callback)
        rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback', ActionFeedback, self.cancel_callback)
        rospy.Subscriber('/goal_failure_count', GoalFailureCountArray, self.goal_failure_count_callback)
        self.goal_failure_count_pub = rospy.Publisher('/goal_failure_count', GoalFailureCountArray, queue_size=10)
        rospy.spin()

    def inspect_callback(self, data):
        if data.name == "inspect":
            self.active_action_id = data.action_id
            self.active_name = data.name
            self.active_wp = data.parameters[1].value

    def goal_failure_count_callback(self, data):
        self.goal_failure_count = {(gfc.goal_predicate, "wp" + str(gfc.waypoint)): gfc.fail_count for gfc in data.goal_failure_count_array}

    def status_callback(self, data):
        if data.action_id == self.active_action_id and self.active_name == "inspect" and data.status == 2:
            try:
                time_over = inputimeout(prompt="Photo taken of " + self.active_wp + ". Press ENTER to reject or wait to approve photo.", timeout=10)
                print("\nPhoto rejected. Re-adding " + self.active_wp + " as goal and replanning.")
                self.remove_predicate(self.active_wp)
                self.should_cancel_dispatch = True
                
                predicate_wp = ("photographed", self.active_wp)
                if predicate_wp in self.goal_failure_count:
                    self.goal_failure_count[predicate_wp] += 1
                else:
                    self.goal_failure_count[predicate_wp] = 1
                # Publish the updated failure count
                goal_failure_count_array = GoalFailureCountArray()
                goal_failure_count_array.goal_failure_count_array = [
                    GoalFailureCount(goal_predicate=predicate, waypoint=int(wp[2:]), fail_count=count)
                    for (predicate, wp), count in self.goal_failure_count.items()
                ]
                self.goal_failure_count_pub.publish(goal_failure_count_array)

                if self.goal_failure_count[predicate_wp] >= 3:
                    rospy.logwarn("Goal '{}' has failed 3 times. Removing it from the knowledge base.".format(predicate_wp))
                    self.remove_goal(self, predicate_wp)
            
            except TimeoutOccurred:
                time_over = ("\nPhoto approved, continuing mission.")
                print(time_over)
    
    def remove_goal(self, goal_name):
        goal_predicate, wp = goal_name
        values = [KeyValue("wp", wp)]
        kb_update_service = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
        req = KnowledgeUpdateServiceRequest(update_type=3, knowledge=KnowledgeItem(knowledge_type=1, instance_type='', instance_name='', attribute_name=goal_predicate, values=values))
        kb_update_service(req)

    def cancel_callback(self, data):
        if self.should_cancel_dispatch and data.action_id == self.active_action_id + 1 and data.status == 2:
            self.cancel_dispatch(data, self.active_action_id)
            self.should_cancel_dispatch = False

    def remove_predicate(self, wp):
        rospy.wait_for_service('/rosplan_knowledge_base/update')
        update_kb = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
        knowledge = KnowledgeItem()
        knowledge.knowledge_type = KnowledgeItem.FACT
        knowledge.attribute_name = "photographed"
        knowledge.values = [KeyValue("wp", wp)]
        try:
            update_kb(update_type=2, knowledge=knowledge)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed:", e)

    def cancel_dispatch(self, data, action_id):
        if data.action_id == action_id+1 and data.status == 2:
            rospy.wait_for_service('/rosplan_plan_dispatcher/cancel_dispatch')
            cancel = rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch', Empty)
        
            
            try:
                cancel()
                print("Dispatch canceled. Replanning.")
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed:", e)

if __name__ == '__main__':
    try:
        InspectionManager()
    except rospy.ROSInterruptException:
        pass
