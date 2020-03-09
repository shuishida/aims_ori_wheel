#! /usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
	pub = rospy.Publisher('phrases', String, queue_size=10)
	rospy.init_node('important_phrase_generator')
	r = rospy.Rate(10) 
	while not rospy.is_shutdown():
	   pub.publish("hello world")
	   r.sleep()
