#!/usr/bin/env python

# A simple node that takes in the cmd_vel topic and calculates a power consumption from its x component.
# Power consumption is published for the battery plugin to use.

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


def talker(data):
    pub = rospy.Publisher('battery/consumer/0', Float32, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    pub.publish(5*abs(data.linear.x)) # Ajust to fit wanted power consumption
    rate.sleep()


def listener():
    rospy.init_node('battery_monitor', anonymous=False)
    rospy.Subscriber("cmd_vel", Twist, talker)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()