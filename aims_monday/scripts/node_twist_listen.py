#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

def callback(data):
    rospy.loginfo("Heard twist: ")
    rospy.loginfo(msg)


if __name__ == "__main__":
    rospy.init_node('twist_listener')
    rospy.Subscriber('cmd_vel', Twist, callback)
    rospy.spin()
