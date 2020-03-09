#! /usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolRequest


if __name__ == "__main__":
    rospy.wait_for_service("receive_bool")
    set_bool = rospy.ServiceProxy("receive_bool", SetBool)

    response = set_bool(True)
    print(response)
