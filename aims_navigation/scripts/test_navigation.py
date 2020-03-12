#! /usr/bin/env python
import rospy
import actionlib
from aims_msgs.msg import *
from geometry_msgs.msg import PoseStamped

def navigation_client():
    client = actionlib.SimpleActionClient('navigation', NavigateAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    goal = NavigateGoal()
    goal.goal.position.x = 6.
    goal.goal.position.y = 4.8
    client.send_goal(goal)
    client.wait_for_result()
    print(client.get_result())

if __name__ == '__main__':
    rospy.init_node('test_client')
    res = navigation_client()
    rospy.spin()
