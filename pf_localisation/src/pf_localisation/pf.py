#!/usr/bin/env python
""" File provides a subclass with which to implement the particle filter.

Maintainer: Charlie Street
"""

import os
import sys
from geometry_msgs.msg import Pose, PoseArray, Quaternion, \
    Point, PoseWithCovarianceStamped
from numpy.random.mtrand import vonmises

from pf_base import PFLocaliserBase
import math
import rospy
import tf  # for converting to and from quaternions

from util import rotateQuaternion, getHeading
from random import random, randrange, gauss, shuffle
import time


class PFLocaliser(PFLocaliserBase):
    """ Subclass of PFLocaliserBase, implement all un-implemented functions! """
    def __init__(self):

        # Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # NOTE: Changing these values will effect the performance!
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.00006081294 # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.00007881588 # Odometry x axis (fwd) noise
        self.ODOM_DRIFT_NOISE = 0.0003284  # Odometry y axis (side-side) noise

        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 50  # Number of readings to predict

        # Generating particles
        # NOTE: The number of particles will be needed in the functions you 
        # implement. Use and adjust this value to see the effect on performance!
        self.NUMBER_PARTICLES = 500

        # NOTE: You will need these to generate the initial particle cloud. 
        # This is when we use the initial pose as the centre of a Gaussian.
        # Use and adjust these values to see the effect
        self.POSITION_STANDARD_DEVIATION = 0.05
        self.ORIENTATION_STANDARD_DEVIATION = 0.05


    def point_to_cell(self, point):
        """ Converts a point into a cell location.
        Args:
            point (geometry_msgs.msg.Point) point using meter units
        Return:
            (int, int) cell location on map
        """
        res = self.occupancy_map.info.resolution
        return (int(math.floor(point.x / res)), int(math.floor(point.y / res)))


    def is_valid_position(self, point):
        """ Checks whether a point is a valid location for the robot on the map.
        Args:
            point (geometry_msgs.msg.Point) point using meter units
        Return:
            (boolean) whether or not the robot can be in that position
        """
        if point is None: return False
        (x, y) = self.point_to_cell(point)
        width = self.occupancy_map.info.width
        height = self.occupancy_map.info.height
        index = x + y * width

        # data is row major
        not_valid = (index > len(self.occupancy_map.data) or
                     x >= width or x < 0 or y >= height or y < 0)
        value = -1 if not_valid else self.occupancy_map.data[index]
        return value != -1


    def generate_random_pose(self):
        """ Generates a random pose on the map.
        Return:
            (geometry_msgs.msg.Pose) the random pose on the map
        """
        p = Pose() # Instantiate pose
        while not self.is_valid_position(p.point): # Repeat until unoccupied point is found
            p.point.x = random() * self.occupancy_map.info.width # Sample x location on map
            p.point.y = random() * self.occupancy_map.info.heigth # Sample y location on map
        p.point.z = 0.0 # No elevation, z-coordinate is 0
        p.orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, 2*math.pi*random()) # Random rotation around z-axis (vertical axis)
        return p # Return pose # NotImplementedError("generate_random_pose not implemented!")


    def initialise_particle_cloud(self, initial_pose):
        """ Set particle cloud to initial pose plus noise.

        Note that the number of particles you have in the cloud is an
        important parameter to choose!

        There are two ways you can implement this:
        1. Generate a completely random set of poses at valid positions.
        2. Use the initial pose and generate points at some Gaussian
           around that pose.

        I'd recommend using self.POSITION_STANDARD_DEVIATION and
                            self.ORIENTATION_STANDARD_DEVIATION here!

        Args:
            initial_pose: The initial pose provided

        Returns:
            pos_array: A list of poses representing the particle cloud
        """
        # 1st approach
        p = [self.generate_random_pose(self) for i in xrange(self.NUMBER_PARTICLES)] # Must use PoseArray() instead?
        # 2nd approach
        def self.generate_gaussian_pose(self, initial_pose):
            p = Pose()
            p.point.x = gauss(initial_pose.point.x, self.POSITION_STANDARD_DEVIATION)
            p.point.y = gauss(initial_pose.point.y, self.POSITION_STANDARD_DEVIATION)
            p.point.z = initial_pose
            initial_yaw = tf.transformations.euler_from_quaternion(initial_pose.orientation)[2] # Convert initial orientation from quaternion into euler representation and get yaw angle
            rand_yaw = vonmises(initial_yaw, self.ORIENTATION_STANDARD_DEVIATION) # Generate random yaw angle
            p.orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, rand_yaw) # Convert yaw angle into quaternion representation
        p = [self.generate_gaussian_pose(initial_pose) for i in xrange(self.NUMBER_PARTICLES)] # Must use PoseArray() instead?
        return p # Return list of poses # NotImplementedError("initialise_particle_cloud not implemented!")


    def update_particle_cloud(self, scan):
        """ Update particle cloud given laser scan.

        This function requires using the sensor model to compute the likelihoods
        of particles (odometry prediction is already written and used
        elsewhere).

        Before computing this, you may want to add some Gaussian noise to the 
        particles due to noise in the action model. If you do this, make sure it
        is a valid position though!

        Given the likelihoods, you then want to resample a new set of particles.
        You may also wish to add a small amount of Gaussian noise to the newly
        generated particles.

        Note that you will need to use the sensor model here.
        The function you will want to compute the likelihood is
        self.sensor_model.get_weight(scan, particle).

        This function should set self.particlecloud.poses to the new pose list.

        Args:
            scan: The LaserScan message
        """
        # Samples (particles) are drawn from the original distribution
        # The particles are driven through the nonlinear function
        # Each particle is weighted with an importance factor that incorporates the knowledge of the measurement
        w = [self.sensor_model.get_weight(scan, particle) for particle in self.particlecloud.poses] # Compute the likelihood weighting for each of a set of particles.
        posterior = prior * likelihood
        normalize(posterior)
        # These important factors are used to choose a new set of particles that appropriately represents the a posteriori probability density function (resampling)
        
        self.particlecloud.poses = 
        return # NotImplementedError("update_particle_cloud not implemented!")


    def estimate_pose(self):
        """ Create new estimated pose, given particle cloud.

        E.g. just average the location and orientation values of each of
        the particles and return this.

        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after
        throwing away any which are outliers.

        Remember that your particle cloud poses are stored in
        self.particlecloud.poses

        Return:
            pose: The estimated pose (should be a geometry_msgs.Pose message)
        """
        estimate = Pose() # Instantiate pose estimate
        estimate.point = np.mean(self.particlecloud.poses.point) # Average points (position)
        def avg_quaternion(self, Q):
        """ Q is an Mx4 matrix of quaternions. Q_avg is the average quaternion.
        Based on
        Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman. 
        "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30, 
        no. 4 (2007): 1193-1197. """
            A = np.zeros((4,4)) # Form symmetric accumulator matrix
            M = Q.shape[0]
            for i in range(M)
                q = Q[i,:]
                if q[0] < 0: # Handle antipodal configuration
                    q = -q
                A = np.outer(q, q) + A # Rank 1 update
            A = (1.0 / M) * A # Scale
            # Get eigenvector corresponding to largest eigenvalue # [Qavg, Eval] = eigs(A,1)
            vals, vects = np.linalg.eig(A)
            maxcol = list(vals).index(max(vals))
            Q_avg = vects[:,maxcol]
            return Q_avg
        estimate.orientation = self.avg_quaternion(self.particlecloud.poses.orientation) # Average quaternions (orientation)
        return estimate # NotImplementedError("estimate_pose not implemented!")
