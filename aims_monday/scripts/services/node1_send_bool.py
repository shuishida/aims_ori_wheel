#! /usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolRequest
import argparse


parser = argparse.ArgumentParser()

parser.add_argument("-b", action="store_true")

args = parser.parse_args()


if __name__ == "__main__":
    rospy.wait_for_service("receive_bool")
    set_bool = rospy.ServiceProxy("receive_bool", SetBool)

    response = set_bool(args.b)
    print(response)
