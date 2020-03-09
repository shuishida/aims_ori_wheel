#!/usr/bin/env python

import rospy
from std_msgs.msg import String

if __name__ == "__main__":
	pub = rospy.Publisher('phrases', String)
	rospy.init_node('speaker')
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish('Hello AIMS.')
		r.sleep()


