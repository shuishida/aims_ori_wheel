#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from potential_flow import get_grads
import numpy as np

class APFController(object):
    """ class to implement safety controller by overriding the velocities
    given by the PS4 controller. """

    def __init__(self):
        """ perform subscriptions and initialise publisher to the velocity topic
        which will override the PS4 controller
        """

        rospy.init_node('apf_controller')

        self.scan = rospy.Subscriber('/front/scan', LaserScan, self.get_grads, queue_size=1)

        self.vel_pub = rospy.Publisher('joy_teleop/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber('/bluetooth_teleop/joy', Joy, self.store_joystick, queue_size=1)

        self.x_button = 0

        rospy.spin()

    def store_joystick(self, data):
        """ stores whether the x button is pressed on the PS4 controller. """

        self.x_button = data.buttons[0]

    def get_grads(self, msg):
        force = get_grads(msg.ranges, msg.angle_min, msg.angle_max, k=10, thresh=0.3, ignore_thresh=0.005):


    def override_control(self, obstacle_detected):
        """ Function to override the control given by the PS4 controller. If the
        deadmans switch on the controller is not pressed the safety controller
        should not be activated. Otherwise, if an obstacle is detected this
        function should publish an override velocity to joy_teleop/cmd_vel

        Args:
            obstacled_detected: a ROS bool message describing whether an
                obstacle has been detected.
        """

        # if the x button is not pressed on the PS4 controller do not modify the
        # velocity.
        if self.x_button != 1:
            return


        rospy.loginfo(obstacle_detected.data)
        if obstacle_detected.data:
            self.vel_pub.publish(Twist())


if __name__ == '__main__':
    APFController()
