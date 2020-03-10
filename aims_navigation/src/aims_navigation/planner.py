#! /usr/bin/env python

import rospy
import math
import numpy as np
from scipy.spatial import KDTree
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose

class Planner(object):
    """ class for planning a path from the robot location to a goal location. """

    def __init__(self):
        """ initialise the class by waiting for a costmap and performing any
        necessary precomputation. """

        # wait for costmap
        self.map_received = False
        rospy.Subscriber("/map/costmap/costmap", OccupancyGrid, self.store_costmap)

        while not self.map_received:
            rospy.sleep(2.)
            rospy.loginfo('waiting for cost map...')
        rospy.loginfo("cost map received.")

        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

        ##################################################################
        #************** execute any required precomputation. ************#
        ##################################################################

    def plan(self, goal_pose):
        """ generates a plan from the start pose to the goal pose. To determine
        the start pose the function waits for a message of type estimatedpose
        which will be provided by your localisation system. For testing purposes
        this may be replaced by a suitable start pose.

        Args:
            goal_pose: a ROS pose message specifying the desired final pose of
            the robot.
        Returns:
            result: a boolean describing whether the planner succeeded or failed
            plan: a list of tuples of defining the path of the format
                [(x1, y1), (x2, y2),..., (xN, yN)]
        """

        # wait for the current pose. the plan will start at the current pose.
        try:
            start_pose = rospy.wait_for_message(
                                "estimatedpose", PoseStamped, timeout=5)
        except:
            rospy.loginfo('Did not receive current pose.')
            return False, []

        ##############################################################
        #********** compute the plan and the result here ************#
        ##############################################################
        result, plan = True, [(0.5, 0.5), (1.2, 1.5), (2.1, 2.2)]

        # publish the plan so that it can be visualised on rviz
        self.publish_path(plan)
        return result, plan

    def publish_path(self, pts):
        """ publishes the current path to view in rviz.

        Args:
            pts: a list of tuples defining the path of the format
                [(x1, y1), (x2, y2),..., (xN, yN)]
        """

        path = Path()
        path.header.frame_id = 'map'
        for pt in pts:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.header.frame_id = 'map'
            path.poses.append(pose)
        self.path_pub.publish(path)

    def store_costmap(self, costmap):
        """ callback for storing the costmap. stores relevant information in
        variables to be used later.

        Args:
            costmap: a ROS message of type OccupancyGrid
        """

        self.map_received = True
        self.min_x = costmap.info.origin.position.x
        self.min_y = costmap.info.origin.position.y
        self.max_x = self.min_x + costmap.info.width * costmap.info.resolution
        self.max_y = self.min_y + costmap.info.height * costmap.info.resolution
        self.data = np.reshape(costmap.data, (costmap.info.height, costmap.info.width))
        self.res = costmap.info.resolution


    def sample_points(self, n_sample):
        """ randomly sample a number of points which are in free space.

        Args:
            n_sample: the number of points to sample.

        Returns:
            points: a list of tuples of x-y points which are in free space.
        """

        points = []

        while len(self.x_points) < n_sample:
            sx = np.random.uniform(self.min_x, self.max_x)
            sy = np.random.uniform(self.min_y, self.max_y)

            if self.free_space(sx, sy):
                points.append((sx, sy))
        return points

    def is_collision(self, pt1, pt2):
        """ checks if the line connecting two points passes through an space
        which is occupied.

        Args:
            pt1: a tuple of the form (x1, y1) where x1 and y1 are floats in
                metres
            pt2: a tuple of the form (x2, y2) where x2 and y2 are floats in
                metres

        Returns:
            result: true if the line connecting the points passes through
                occupied space. false if the line is in free space.
        """

        x, y = pt1[0], pt1[1]
        nx, ny = pt2[0], pt2[1]
        in_collision = False

        # step between points at map resolution
        dx = nx - x
        dy = ny - y
        dist = math.hypot(dx, dy)
        steps = int(round(dist/self.res))
        for step in range(steps):
            chk_x = x + dx*step/steps
            chk_y = y + dy*step/steps

            # if any point between the two points is not in free space this
            # line is in collision
            if not self.free_space(chk_x, chk_y):
                in_collision = True
                return in_collision

        return in_collision


    def free_space(self, x, y):
        """ checks whether a point is in free space according to the cost map.

        Args:
            x: float defining x location in metres
            y: float defining y location in metres

        Returns:
            result: true if this point is in free space and false otherwise
        """

        # compute the index in the costmap array
        row, col = self.metre_to_index(x, y)
        val = self.data[row][col]

        # if 0 in the costmap this is freespace otherwise it is occupied
        if val == 0:
            return True
        else:
            return False

    def metre_to_index(self, x, y):
        """ converts an xy point in metres to the index in the costmap array

        Args:
            x: float defining x location in metres
            y: float defining y location in metres

        Returns:
            row: corresponding row in the occupancy grid
            col: corresponding column in the occupancy grid
        """

        row = int(np.floor((y - self.min_y)/self.res))
        col = int(np.floor((x - self.min_x)/self.res))
        return row, col
