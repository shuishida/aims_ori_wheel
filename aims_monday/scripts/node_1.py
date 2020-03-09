#! /usr/bin/env python
import rospy
from std_msgs.msg import String


if __name__ == "__main__":
    print('hello world')
    rospy.init_node('speaker')

    pub = rospy.Publisher('phrases', String)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish('hello world')
        r.sleep()
