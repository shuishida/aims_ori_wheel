#! /usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def handle_bool(req):
	# Request is of type SetBoolRequest
	print('Received %s' % req.data)
	return SetBoolResponse(success=True, message='Received %s' % req.data)
	

def print_bool_server():
	s = rospy.Service('receive_bool', SetBool, handle_bool)
	rospy.spin() # Make server exist forever

if __name__ == "__main__":
	rospy.init_node('print_bool_server')
	print_bool_server()
