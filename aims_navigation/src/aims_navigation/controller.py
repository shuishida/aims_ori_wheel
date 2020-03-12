#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped, Pose, Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import rospy

class Controller(object):
    """ class to control the robot to follow a path and reach a desired pose.
    """

    def __init__(self):
        """ subscribe to topics and create a velocity publisher
        """

        rospy.Subscriber("estimatedpose", PoseStamped, self.store_pose)
        rospy.Subscriber("front/scan", LaserScan, self.store_laser)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 5)

        # declare constants
        self.near_goal_tol = 0.3
        self.lookahead_dist = 1.

    def store_pose(self, data):
        """ store the current pose """

        self.current_pose = data.pose

    def store_laser(self, data):
        """ store the current laser scan """

        self.current_laser = data

    def control(self, path, goal_pose, check_preempted):
        """ executes control of the robot until goal reached. The robot should
        follow the requested path. Once the final location is reached the robot
        should turn to match the orientation given in the goal pose.

        Args:
            path: a list of tuples specifying the path to be followed in the
                format [(x1, y1), (x2, y2), ... , (xN, yN)]
            goal_pose: a ROS pose message specifying the desired final pose.
            check_preempted: a function handle which returns true if the action
                has been preempted.

        Returns:
            success: returns true if the controller succeeded and false
                otherwise.
            preempted: returns true if the controller has been preempted and
                false otherwise.
        """

        rospy.loginfo('executing plan...')
        preempted = False
        success = False
        goal = np.array((goal_pose.position.x, goal_pose.position.y))

        # publish controls at 10hz until near goal
        near_goal = False
        r = rospy.Rate(10)
        while not near_goal:

            # get a local goal to follow the path
            local_goal = self.get_local_goal(path)

            ##################################################################
            #************* implement compute_vel to calc velocity ***********#
            ##################################################################
            cmd = self.compute_vel(local_goal)
            self.vel_pub.publish(cmd)
            r.sleep()

            # check if goal reached
            cur_pose = np.array((self.current_pose.position.x, self.current_pose.position.y))
            if np.linalg.norm(abs_goal - cur_pose) < self.near_goal_tol:
                near_goal = True
                rospy.loginfo('position reached')

            # if the goal has been preempted then exit
            if check_preempted():
                preempted = True
                return success, preempted

        # make the robot face in the correct orientation
        near_orientation = False
        while not near_orientation:

            ##################################################################
            #****implement code to publish an angular velocity until the*****#
            #***********robot is facing near the desired orientation*********#
            ##################################################################

            if check_preempted():
                preempted = True
                return success, preempted

        success = True
        return success, preempted

    def compute_vel(self, local_goal):
        """ compute the current velocity command for the robot based on the
        local goal. For a robust solution the current laser signal should
        also be considered to ensure that the robot can avoid obstacles.

        Args:
            local_goal: tuple (x, y) specifying the point the robot should
                move towards.

        Returns:
            cmd: a ROS twist message specifying an appropriate velocity command
        """

        cmd = Twist()
        return cmd

    def pose_to_yaw(self, pose):
        """ compute the yaw in -pi to pi given a ROS pose message.

        Args:
            pose: a ROS pose message.

        Returns:
            yaw: the yaw in the range [-pi, pi]
        """

        yaw = 2.0*math.acos(pose.orientation.w)
        if pose.orientation.z < 0:
            yaw = yaw*-1
        return yaw

    def compute_yaw_error(self, targ_yaw, yaw):
        """ computes the yaw error between a target yaw and current yaw in the
        range [-pi, pi]

        Args:
            targ_yaw: the desired yaw in radians
            yaw: the current yaw in radians
        Returns:
            yaw_error: the """

        yaw_error = targ_yaw - yaw
        yaw_error = (yaw_error + math.pi) % (2.0*math.pi) - math.pi
        return yaw_error

    def get_local_goal(self, path):
        """ Finds an appropriate point on the path to make the robot move
        currently this simply looks for the point furthest along the path
        which is within self.lookahead_dist of the robot. If no point is within
        this distance it returns the closest point to the robot.

        Args:
            path: list of tuples of xy points specifying the path to be
                followed

        Returns:
            local_goal: a tuple in the form (x, y) specifying the local goal
                the robot should move towards.
        """

        goal_point = None
        closest_dist = float('inf')
        cur_pose = np.array((self.current_pose.position.x, self.current_pose.position.y))

        # check each point
        for point in path:
            point = np.array(point)

            # store the point furthest along the path within the lookahead dist
            if np.linalg.norm(point - cur_pose) < self.lookahead_dist:
                goal_point = point

            # also store the point closest to the robot
            if np.linalg.norm(point - cur_pose) < closest_dist:
                closest_dist = np.linalg.norm(point - cur_pose)
                closest_point = point

        # if we didn't find a point within the lookahead dist then use closest
        # point to robot
        if goal_point is None:
            goal_point = closest_point

        return tuple(goal_point)
