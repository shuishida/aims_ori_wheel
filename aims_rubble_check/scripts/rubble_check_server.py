#!/usr/bin/env python
""" Action server for checking rubble qr code.

Author: Charlie Street
Owner: Charlie Street
"""

from aims_rubble_check.msg import RubbleCheckAction, RubbleCheckResult
from std_msgs.msg import String
from threading import Lock, Condition
import actionlib
import rospy
import time

TIMEOUT = 1

class RubbleCheckServer(object):
    """ Class for rubble check action server.

    Attributes:
        _lock: For locking access to qr messages
        _condition: Condition variable
        _last_rubble: Last rubble message received
        _as: The action server
        _qr_sub: The subscriber to the qr code topic
    """

    def __init__(self, action_server_name):
        """ Creates and starts the action server.

        Args:
            action_server_name: The name of the action server.
        """

        self._lock = Lock()
        self._condition = Condition(self._lock)
        self._last_rubble = None
        self._qr_sub = rospy.Subscriber('/qr_codes',
                                        String,
                                        callback=self.qr_cb,
                                        queue_size=5)

        self._as = actionlib.SimpleActionServer(action_server_name,
                                                RubbleCheckAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)

        self._as.start()
        rospy.loginfo('Rubble Check Server started...')


    def execute_cb(self, goal):
        """ Checks qr topic until something is called or timeout reached.

        Args:
            goal: Goal message (empty)
        """
        self._condition.acquire()
        self._last_rubble = None
        self._condition.release()

        start = time.time()
        self._condition.acquire()
        while ((self._last_rubble is None) and
               (time.time() - start < TIMEOUT)):
            self._condition.wait(timeout=TIMEOUT)
        self._condition.release()

        result = RubbleCheckResult()
        self._condition.acquire()
        if self._last_rubble is None:
            result.rubble_size = 'large'
        else:
            if 'small' in self._last_rubble:
                result.rubble_size = 'small'
            elif 'medium' in self._last_rubble:
                result.rubble_size = 'medium'
            else:
                result.rubble_size = 'large'
        self._condition.release()

        self._as.set_succeeded(result)


    def qr_cb(self, msg):
        """ Callback for qr topic.

        Args:
            msg: A string with decoded qr code
        """
        self._condition.acquire()
        if 'rubble' in msg.data:
            self._last_rubble = msg.data
        self._condition.release()

if __name__ == '__main__':
    rospy.init_node('rubble_check')
    server = RubbleCheckServer(rospy.get_name())
    rospy.spin()
