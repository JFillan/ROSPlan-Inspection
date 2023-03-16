#!/usr/bin/env python
import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from inputimeout import inputimeout, TimeoutOccurred
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
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

        rospy.init_node('inspect_subscriber')
        rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch', ActionDispatch, self.inspect_callback)
        rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback', ActionFeedback, self.status_callback)
        rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback', ActionFeedback, self.cancel_callback)
        rospy.spin()

    def inspect_callback(self, data):
        if data.name == "inspect":
            self.active_action_id = data.action_id
            self.active_name = data.name
            self.active_wp = data.parameters[1].value

    def status_callback(self, data):
        if data.action_id == self.active_action_id and self.active_name == "inspect" and data.status == 2:
            try:
                time_over = inputimeout(prompt="Photo taken of " + self.active_wp + ". Press ENTER to reject or wait to approve photo.", timeout=10)
                print("\nPhoto rejected. Re-adding " + self.active_wp + " as goal and replanning.")
                self.remove_predicate(self.active_wp)
                self.should_cancel_dispatch = True
            except TimeoutOccurred:
                time_over = ("\nPhoto approved, continuing mission.")
                print(time_over)

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
            print("Service call failed:", e)

    def cancel_dispatch(self, data, action_id):
        if data.action_id == action_id+1 and data.status == 2:
            rospy.wait_for_service('/rosplan_plan_dispatcher/cancel_dispatch')
            cancel = rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch', Empty)
            try:
                cancel()
                print("Dispatch canceled. Replanning.")
            except rospy.ServiceException as e:
                print("Service call failed:", e)

if __name__ == '__main__':
    try:
        InspectionManager()
    except rospy.ROSInterruptException:
        pass
