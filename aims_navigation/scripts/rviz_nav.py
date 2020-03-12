#! /usr/bin/env python

import rospy
import actionlib
from aims_navigation.controller import Controller
from aims_navigation.planner import Planner
from aims_msgs.msg import *
from geometry_msgs.msg import *

def callback(pose_stamped):
    """ send goals from rviz to the navigate action server """

    client = actionlib.SimpleActionClient('navigation', NavigateAction)
    client.wait_for_server()
    goal = NavigateGoal()
    goal.goal = pose_stamped.pose
    client.send_goal(goal)

def listener():
    rospy.init_node('rviz_node', anonymous=True)
    rospy.Subscriber("navigate_simple/goal", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
