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
        self.near_goal_tol = 0.25
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
        rate = rospy.Rate(10)
        while not near_goal:

            # get a local goal to follow the path
            local_goal = self.get_local_goal(path)

            ##################################################################
            #************* implement compute_vel to calc velocity ***********#
            ##################################################################
            cmd = self.compute_vel(local_goal)
            self.vel_pub.publish(cmd)
            rate.sleep()

            # check if goal reached
            cur_pose = np.array((self.current_pose.position.x, self.current_pose.position.y))
            if np.linalg.norm(goal - cur_pose) < self.near_goal_tol:
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
            max_err = 5.0 / 180.0 * math.pi # Maximum tolerated heading error
            # P controller
            k = 0.3/1.5 # Controller gain
            y = self.pose_to_yaw(self.current_pose) # Measurement (actual heading)
            r = self.pose_to_yaw(goal_pose) # Reference (desired heading)
            e = self.compute_yaw_error(r, y) # Error
            if abs(e) <= max_err:
                near_orientation = True
                rospy.loginfo('orientation reached')
            else:
                u = k * e # Control signal (angular velocity)
                cmd = Twist() # Create message
                cmd.angular.z = u
                self.vel_pub.publish(cmd)
                rate.sleep()

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
        k_omega = 0.2/2.0 # Controller gain for angular velocity
        k_v = 0.5/3.0 # Controller gain for velocity
        dist_thres_goal = 1.0 # Distance threshold for switching between quadratic and conical attractive potential
        zeta = 10.0 # Gain for attractive potential
        dist_thres_obstacle = 0.6*32.5 # Distance threshold for switching to zero repulsive potential
        eta = 1.0*1.0 # Gain for repulsive potential
        pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        pos_goal = np.asarray(local_goal) # Coordinates of goal
        dist_goal = np.linalg.norm(pos - pos_goal) # Euclidean distance between robot and goal
        if dist_goal <= dist_thres_goal: # Near to goal
            F_world = - zeta * (pos - pos_goal) # Attractive force from quadratic potential
        else: # Far from goal
            F_world = - dist_thres_goal * zeta * (pos - pos_goal) / dist_goal # ...from conical potential

        #print("F")
        #print(pos)
        #print(pos_goal)
        #print(dist_goal)
        #print(F_world)
        
        # Trafo from world frame into robot frame
        robot_yaw = - self.pose_to_yaw(self.current_pose) # Angle between world fram and robot frame
        F = np.array([np.cos(robot_yaw) * F_world[0] - np.sin(robot_yaw) * F_world[1], np.sin(robot_yaw) * F_world[0] + np.cos(robot_yaw) * F_world[1]]) # Force in robot frame

        #print(robot_yaw)
        #print(F)

        # F_mag = np.linalg.norm(F)
        # F_angle = np.arctan2(pos_goal[1] - pos[1], pos_goal[0] - pos[0])
        
        view_range = 1.0
        ignore_thresh = 0.05
        
        ranges = np.array(self.current_laser.ranges)
        
        angles = np.linspace(self.current_laser.angle_min, self.current_laser.angle_max, len(ranges))
        ranges_temp = ranges[np.where(angles >= -view_range, angles <= view_range, False)]
        angles_temp = angles[np.where(angles >= -view_range, angles <= view_range, False)]
        ranges = ranges_temp[ranges_temp >= ignore_thresh]        
        angles = angles_temp[ranges_temp >= ignore_thresh]
        
        # is_obst = ranges.min() <= thresh
        min_ind = np.argmin(ranges)
        dist_obstacle = ranges[min_ind]
        theta = angles[min_ind]
        
        # self.current_laser        
        
        #for i in range(ranges.shape[0]):
        # dist_obstacle = ranges[i]
        # dist_obstacle = np.linalg.norm(pos - pos_obstacle)
        # dist_obstacle = r

        grad_dist_obstacle = 1.0 # ToDo

        if dist_obstacle <= dist_thres_obstacle:
            F_mag = eta * (1.0/dist_thres_obstacle - 1.0/dist_obstacle) * 1.0/(dist_obstacle**2) * grad_dist_obstacle

            # theta = angles[i]
            F[0] = F[0] + F_mag * np.cos(theta)
            F[1] = F[1] + F_mag * np.sin(theta)
        
        # F = F_att + F_rep
        # F = - gradient(U)
        print(F)
        omega = - k_omega * self.compute_yaw_error(0.0, np.arctan2(F[1], F[0]))
        #print("Angular velocity:")
        print(omega)
        v = k_v * max(0.0, F[0]) # np.linalg.norm(F)
        #print("Velocity:")
        #print(v)
        cmd = Twist()
        cmd.linear.x = min(v, 0.3)
        cmd.angular.z = omega
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
