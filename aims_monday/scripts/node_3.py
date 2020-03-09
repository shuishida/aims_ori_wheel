#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

flag = False

def handle_bool(req):
	# Request is of type SetBoolRequest
	print('Received %s' % req.data)
	flag = req.data
	return SetBoolResponse(success=True, message='Received %s' % req.data)

if __name__ == "__main__":
	s = rospy.Service('receive_bool', SetBool, handle_bool)
	#rospy.spin()
	pub = rospy.Publisher('phrases', String)
	rospy.init_node('speaker')
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		if flag:
			pub.publish('Hello AIMS.')
			r.sleep()

	
