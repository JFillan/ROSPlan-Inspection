#!/usr/bin/env python
import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from inputimeout import inputimeout
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService
from diagnostic_msgs.msg import KeyValue
from std_srvs.srv import Empty


def inspect_callback(data):
    if data.name == "inspect":
        wp = data.parameters[1].value
        action_id = data.action_id
        name = data.name
        rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback', ActionFeedback, lambda data: status_callback(data, action_id, name, wp))

def status_callback(data, action_id, name, wp):
    if data.action_id == action_id and name == "inspect" and data.status == 2:
        try:
            time_over = inputimeout(prompt="Photo taken of " + wp + ". Press ENTER to reject or wait to approve photo.", timeout=10)
            print("\nPhoto rejected. Re-adding " + wp + " as goal and replanning.")
            remove_predicate(wp)
            check_status(action_id)
        except Exception:
            time_over = ("\nPhoto approved, continuing mission.")
            print(time_over)  


def remove_predicate(wp):
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    update_kb = rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    knowledge = KnowledgeItem()
    knowledge.knowledge_type = KnowledgeItem.FACT
    knowledge.attribute_name = "photographed"
    knowledge.values=[KeyValue("wp", wp)]
    try:
        resp = update_kb(update_type=2, knowledge=knowledge)
        if resp.success:
            print("Predicate removed from the knowledge base.")
        else:
            print("Failed to remove predicate from the knowledge base.")
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def check_status(action_id):
    rospy.Subscriber('/rosplan_plan_dispatcher/action_feedback', ActionFeedback, lambda feedback: cancel_dispatch(feedback, action_id))

def cancel_dispatch(data, action_id):
    if data.action_id == action_id+1 and data.status == 2:
        print(data.action_id)
        print(action_id)
        print(data.status)
        rospy.wait_for_service('/rosplan_plan_dispatcher/cancel_dispatch')
        cancel = rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch', Empty)
        try:
            cancel()
            print("Dispatch canceled. Replanning.")
        except rospy.ServiceException as e:
            print("Service call failed:", e)

def main():
    rospy.init_node('inspect_subscriber')
    rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch', ActionDispatch, inspect_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
