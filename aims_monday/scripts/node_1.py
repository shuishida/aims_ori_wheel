#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from math import pi

if __name__ == "__main__":
	#pub = rospy.Publisher('phrases', String)
	rospy.init_node('speaker')
	r = rospy.Rate(10)
	#while not rospy.is_shutdown():
	#	pub.publish('Hello AIMS.')
	#	r.sleep()
	msg = Twist()
	msgTurn = Twist()
	msg.linear.x = 0.1
	msgTurn.angular.z = pi / 10.0
	pub = rospy.Publisher('cmd_vel', Twist)
	i = 0
	while i < 30: # not rospy.is_shutdown():
		pub.publish(msg)
		r.sleep()
		i = i + 1
	i = 0
	while i < 100:
		pub.publish(msgTurn)
		r.sleep()
		i = i + 1
	i = 0
	while i < 30:
		pub.publish(msg)
		r.sleep()
		i = i + 1
