#! /usr/bin/env python

import rospy
import actionlib
from aims_msgs.msg import *
from geometry_msgs.msg import *
from aims_navigation.planner import Planner
from aims_navigation.controller import Controller

class NavigateActionServer(object):
    """ class for implementing action server for performing navigation """

    def __init__(self):
        """ initialise the action server """

        rospy.init_node('navigation')
        self.ready = False

        # create an action server which provides the NavigateAction
        self._as = actionlib.SimpleActionServer("navigation",
            NavigateAction, execute_cb=self.nav_to_goal, auto_start=False)
        self._as.start()

        # initialise a planner and controller
        self.planner = Planner()
        self.controller = Controller()
        self.ready = True
        rospy.loginfo('navigate action server started.')

    def nav_to_goal(self, req):
        """ callback function to receive and execute navigation goals.

        Args:
            req: a NavigateGoal object. req.goal is the desired final pose.
        """
        rospy.loginfo('navigation goal received')

        # if we have not finished initialising the planner and controller abort
        # this goal
        if not self.ready:
            rospy.loginfo('not ready to navigate, goal aborted.')
            self._as.set_aborted()
            return

        # attempt to plan to the requested goal pose. if the planner fails
        # abort the action.
        plan_res, path = self.planner.plan(req.goal)
        if not plan_res:
            rospy.loginfo('planner failed to find path, goal aborted.')
            self._as.set_aborted()
            return

        # attempt to perform control to execute the path.
        contr_res, preempted = self.controller.control(
                                path, req.goal, self._as.is_preempt_requested)

        # if the action was preempted or aborted set the result appropriately
        if preempted:
            rospy.loginfo('goal preempted.')
            self._as.set_preempted()
            return

        elif not contr_res:
            rospy.loginfo('controller failed to reach goal, goal aborted.')
            self._as.set_aborted()
            return

        # if the controller succeeded the goal is a success
        rospy.loginfo('successfully reached goal.')
        self._as.set_succeeded(NavigateResult(True))

if __name__ == '__main__':
    server = NavigateActionServer()
    rospy.spin()
