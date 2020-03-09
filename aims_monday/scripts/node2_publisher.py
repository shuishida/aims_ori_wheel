#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse



class Publisher():
    def __init__(self):
        self.pub = rospy.Publisher('phrases', String)
        self.service = rospy.Service("receive_bool", SetBool, self.handle_bool)
        self.bool = False

    def run(self):
        while not rospy.is_shutdown():
            if self.bool:
                self.pub.publish('hello world')
                rospy.Rate(10).sleep()

    def handle_bool(self, req):
        msg = 'received %s' % req.data
        print(msg)
        self.bool = req.data
        return SetBoolResponse(success=True, message=msg)


if __name__ == "__main__":
    rospy.init_node('speaker')
    pub = Publisher()
    pub.run()
