#! /usr/bin/env python
import rospy

from std_srvs.srv import SetBool, SetBoolRequest
import rospy


def set_bool_client(bool=True):
    rospy.wait_for_service('receive_bool')
    try:
        set_bool = rospy.ServiceProxy('receive_bool', SetBool)
        response = set_bool(bool)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    sending = False
    print("Setting %s" % (sending))
    resp = set_bool_client(sending)
    if resp.success:
        print("Setting was successful, message: %s" % resp.message)
    else:
        print("Setting was not successful :(")



    