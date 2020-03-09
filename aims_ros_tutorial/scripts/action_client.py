#! /usr/bin/env python

import rospy
import actionlib
from actionlib.msg import TestAction, TestGoal

def feedback(fb):
    print(fb)

def test_client(goal=20):
    client = actionlib.SimpleActionClient('test_this', TestAction)

    client.wait_for_server()

    goal = TestGoal(goal=goal)

    client.send_goal(goal, feedback_cb=feedback)

    client.wait_for_result()
    # client.wait_for_result(timeout=rospy.Duration(3))
    # client.cancel_all_goals()

    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('action_client_node')
    result = test_client()
    print('result:', result)
