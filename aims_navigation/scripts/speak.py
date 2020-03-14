#! /usr/bin/env python
import pyttsx3

from geometry_msgs.msg import PoseStamped, Pose, Twist
from aims_msgs.msg import *
# from std_msgs.msg import String
import numpy as np
import math
import rospy
import actionlib

class Speak(object):
    """ class to provide action server to announce search result in room
    """

    def __init__(self):
        rospy.init_node('speak')
        # self._qr_sub = rospy.Subscriber('/qr_codes',
        #                                 String,
        #                                 callback=self.qr_cb,
        #                                 queue_size=5)
        # self._target_found = False
        # self._as = actionlib.SimpleActionServer("search_room",
        #     SearchRoomAction, execute_cb=self.search_room, auto_start=False)
        self._as = actionlib.SimpleActionServer("speak",
                                                SearchRoomAction, execute_cb=self.speak, auto_start=False)
        # self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)
        # rospy.Subscriber("estimatedpose", PoseStamped, self.store_pose)

        # self.angular_vel = 0.4
        # self.tol = 0.4
        # self.pose_received = False
        self._as.start()
        rospy.loginfo('speak action server started.')

    # def qr_cb(self, msg):
    #     if 'target' in msg.data:
    #         self._target_found = True

    # def store_pose(self, data):
    #     self.current_pose = data.pose
    #     self.pose_received = True

    def speak(self, req):
        """ announce search result """

        # if not self.pose_received:
        #     rospy.loginfo('Pose not received. cannot search room.')
        #     self._as.set_aborted()
        #     return

        rospy.loginfo('Speak...')

        engine = pyttsx3.init()
        engine.setProperty('rate', 140)
        engine.say('No victim has been observed.')
        engine.runAndWait()

        # self._target_found = False
        # start_yaw = self.pose_to_yaw(self.current_pose)
        # robot_turned = False

        # publish commands until room searched
        # while True:
        #     cmd = Twist()
        #     cmd.angular.z = self.angular_vel
        #     self.vel_pub.publish(cmd)
        #
        #     # calculate current yaw error
        #     yaw_now = self.pose_to_yaw(self.current_pose)
        #     yaw_error = start_yaw - yaw_now
        #     yaw_error = (yaw_error + math.pi) % (2.0*math.pi) - math.pi
        #
        #     # record whether we have turned the robot
        #     if yaw_error > math.pi/2:
        #         robot_turned = True
        #
        #     # if the robot has turned and the error is small, stop
        #     if self._target_found:
        #         rospy.loginfo('Target found.')
        #         self._as.set_succeeded(SearchRoomResult(True))
        #         break
        #
        #     if yaw_error < self.tol and robot_turned:
        #         rospy.loginfo('Target not found.')
        #         self._as.set_succeeded(SearchRoomResult(False))
        #         break
        #
        #     # check if we were preempted
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('Search request preempted.')
        #         self._as.set_preempted()
        #         break

        rospy.sleep(0.1)

    # def pose_to_yaw(self, pose):
    #     """ compute current yaw of vehicle in -pi to pi """
    #
    #     yaw = 2.0*math.acos(pose.orientation.w)
    #     if pose.orientation.z < 0:
    #         yaw = yaw*-1
    #     return yaw

if __name__ == '__main__':
    Speak()
    rospy.spin()
