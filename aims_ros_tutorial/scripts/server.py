#! /usr/bin/env python
import rospy

from std_srvs.srv import SetBool, SetBoolResponse
import rospy

def handle_bool(req):
    print("Recieved [%s]"%(req.data))
    return SetBoolResponse(success=True, message="I received %s"%req.data)

def print_bool_server():
    rospy.init_node('print_bool_server')
    s = rospy.Service('receive_bool', SetBool, handle_bool)
    print "Ready to receive bools."
    rospy.spin()

if __name__ == "__main__":
    print_bool_server()
