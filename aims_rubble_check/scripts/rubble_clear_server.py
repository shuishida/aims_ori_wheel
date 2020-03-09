#!/usr/bin/env python
""" Action server for checking rubble qr code.

Author: Charlie Street
Owner: Charlie Street
"""

from aims_rubble_check.msg import *
from std_msgs.msg import String
import actionlib
import rospy
import time

TIMEOUT = 1

class RubbleClearServer(object):
    def __init__(self):
        self._rubble_detect = actionlib.SimpleActionClient('rubble_detect',
                                                        RubbleDetectAction)
        self._rubble_detect.wait_for_server()

        self._as = actionlib.SimpleActionServer('rubble_clear',
                                                RubbleClearAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        rospy.loginfo('Rubble Clear Server started...')


    def execute_cb(self, goal):
        check_goal = RubbleDetectGoal()
        self._rubble_detect.send_goal(check_goal)
        self._rubble_detect.wait_for_result()

        cleared = self._rubble_detect.get_result().open

        while not cleared:
            time.sleep(0.5)
            self._rubble_detect.send_goal(check_goal)
            self._rubble_detect.wait_for_result()
            cleared = self._rubble_detect.get_result().open

        self._as.set_succeeded(RubbleClearResult(True))


if __name__ == '__main__':
    rospy.init_node('rubble_clear')
    server = RubbleClearServer()
    rospy.spin()
