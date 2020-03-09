#!/usr/bin/env python
""" Action server for checking rubble qr code.

Author: Charlie Street
Owner: Charlie Street
"""

from aims_rubble_check.msg import RubbleDetectAction, RubbleDetectResult
from sensor_msgs.msg import LaserScan
import math
import actionlib
import rospy
import time


class RubbleDetectServer(object):
    def __init__(self):
        """ Creates and starts the action server.
        """
        self._as = actionlib.SimpleActionServer('rubble_detect',
                                                RubbleDetectAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)

        self.dist_thrs = [1.5, 2.] # distance parameters. should be greater than distance
                    # to rubble pile. should be smaller than laser returns
                    # into the room. to improve robustness we look for target
                    # with multiple parameters
        self.search_angle = 0.5 # we will search for the rubble in +- this angle (rads)
        self.laser_count = 10 # number of returns to count to detect gaps
        self.laser_topic = '/front/scan'
        self._as.start()
        rospy.loginfo('Rubble Detect Server started...')


    def execute_cb(self, goal):
        open = self.check_open()
        if open:
            rospy.loginfo("The way is clear")
        else:
            rospy.loginfo("The way is not clear")
        self._as.set_succeeded(result=RubbleDetectResult(open=open))

    def check_open(self):
        """ checks for a rubble pile. a rubble pile is detected if there is
        a gap on the left and rhs of the pile. """

        scan = rospy.wait_for_message(self.laser_topic, LaserScan)

        # try to detect the rubble with multiple distance thresholds
        for dist in self.dist_thrs:
            left_gap = False
            right_gap = False
            middle_object = False

            left_counter = 0
            middle_counter = 0
            right_counter = 0
            for i in range(len(scan.ranges)):
                angle = scan.angle_min+i*scan.angle_increment
                d = scan.ranges[i]
                x = d*math.cos(angle)

                if abs(angle) > self.search_angle:
                    continue

                # if we have not found left gap check for this
                if x > dist and not left_gap:
                    left_counter += 1

                    if left_counter > self.laser_count:
                        left_gap = True

                # if we have not found the middle check for this
                if x < dist and left_gap and not middle_object:
                    middle_counter += 1

                    if middle_counter > self.laser_count:
                        middle_object = True

                if x > dist and left_gap and middle_object:
                    right_counter += 1

                    if right_counter > self.laser_count:
                        right_gap = True

                if (left_gap and right_gap) and middle_object:
                    return False

        # if we didn't find rubble at any threshold, must be clear
        return True

if __name__ == '__main__':
    rospy.init_node('rubble_detect')
    server = RubbleDetectServer()
    rospy.spin()
