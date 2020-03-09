#! /usr/bin/env python

import rospy

import actionlib

from actionlib.msg import TestAction, TestFeedback, TestResult

class TestServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('test_this', TestAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    rate = rospy.Rate(1)
    target = goal.goal
    count = 0
    while count < target and not rospy.is_shutdown():
        if self.server.is_preempt_requested():
            self.server.set_preempted(TestResult(count))
            return

        count += 1
        self.server.publish_feedback(TestFeedback(count))
        rate.sleep()

    self.server.set_succeeded(TestResult(count))


if __name__ == '__main__':
  rospy.init_node('test_action_server')
  server = TestServer()
  rospy.spin()
