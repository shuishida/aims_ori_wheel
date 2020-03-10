#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

class main_class:
	
	def __init__(self):
		self.flag = False
		self.s = rospy.Service('receive_bool', SetBool, self.handle_bool)

	def handle_bool(self, req):
		# Request is of type SetBoolRequest
		print('Received %s' % req.data)
		rospy.loginfo("logging")
		self.flag = req.data
		return SetBoolResponse(success=True, message='Received %s' % req.data)

	def running(self):
		#rospy.spin()
		pub = rospy.Publisher('phrases', String)
		rospy.init_node('speaker')
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			print(self.flag)
			if self.flag:
				pub.publish('Hello AIMS.')
			r.sleep()


if __name__ == "__main__":
	m = main_class()
	m.running()
		
