#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.msg import ActionFeedback

# Dispatch cancel function
def cancel_dispatch(data):
    if data.status == 10:
     cancel_dispatch()
    rospy.logerr("Action failed, canceling dispatch")
    cancel_dispatch_service = rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch', Empty)
    rospy.logerr("Dispatch canceled")
    cancel_dispatch_service()

# Callback function when an action status message is received
def action_status_callback(data):
    if data.status == 10:
     cancel_dispatch()
    
if __name__ == '__main__':
    rospy.init_node('replanning_node') # Initialize the ROS node
    rospy.loginfo("Canceling node initiated") 
    # Subscribe to the action status topic
    rospy.Subscriber("rosplan_plan_dispatcher/action_feedback", ActionFeedback, cancel_dispatch)
    rospy.spin() # Spin to keep the node running

