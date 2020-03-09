#! /usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


def handle_bool(req):
    msg = 'received %s' % req.data
    print(msg)
    return SetBoolResponse(success=True, message=msg)

def print_bool_server():
    s = rospy.Service("receive_bool", SetBool, handle_bool)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("print_bool_server")
    print_bool_server()
