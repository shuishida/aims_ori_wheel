#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(msg):
	rospy.loginfo('I heared: %s' % msg.data)

if __name__ == "__main__":
	rospy.init_node('listener')
	rospy.Subscriber('phrases', String, callback)
	rospy.spin() # Keeps node alive
