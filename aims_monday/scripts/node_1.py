#!/usr/bin/env python

import rospy
# from std_msgs.msg import String
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
	msg_turn = Twist()
	msg.linear.x = 0.1
	msg_turn.angular.z = pi / 10.0
	pub = rospy.Publisher('cmd_vel', Twist)
	for i in range(30): # not rospy.is_shutdown():
		pub.publish(msg)
		r.sleep()
	for i in range (100):
		pub.publish(msg_turn)
		r.sleep()
	for i in range(30):
		pub.publish(msg)
		r.sleep()
	#r.spin()
