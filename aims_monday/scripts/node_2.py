#! /usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(f"I heard: {msg.data}")


if __name__ == "__main__":
    rospy.init_node('listener')
    rospy.Subscriber('phrases', String, callback)
    rospy.spin()
