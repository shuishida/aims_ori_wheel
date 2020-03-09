#!/usr/bin/python

""" This is the main entry point for the particle filter exercise node.
It subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.

Owner: Charlie Street
"""

import rospy
from pf_localisation.pf import PFLocaliser
from pf_localisation.util import getHeading, rotateQuaternion
from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import pf_localisation
from threading import Lock

import sys
from copy import deepcopy

class ParticleFilterLocalisationNode(object):

    def __init__(self):
        """ Initialise publishers, subscribers etc. """

        # Minimum change (m/radians) before publishing new particle cloud / pose
        self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.05)

        # This object does most of the work!
        self._particle_filter = PFLocaliser()

        self._latest_scan = None
        self._last_published_pose = None
        self._last_odometry = None
        self._latest_odometry = None
        self._init_pose_received = False

        # Pose publishers
        self._pose_publisher = rospy.Publisher("/estimatedpose",
                                               PoseStamped,
                                               queue_size=100)
        self._amcl_pose_publisher = rospy.Publisher("/amcl_pose",
                                                    PoseWithCovarianceStamped,
                                                    queue_size=100)

        # ROS publisher for the particle cloud
        self._cloud_publisher = rospy.Publisher("/particlecloud",
                                                PoseArray,
                                                queue_size=100)

        # ROS publisher for transforms
        self._tf_publisher = rospy.Publisher("/tf",
                                             tfMessage,
                                             queue_size=100)

        # Get the map
        rospy.loginfo("Waiting for a map...")
        try:
            ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a" +
                         " map_server running: " +
                         "rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))
        self._particle_filter.set_map(ocuccupancy_map)

        # We need to receive laser data
        self._laser_subscriber = rospy.Subscriber("/front/scan", LaserScan,
                                                  self._laser_callback,
                                                  queue_size=1)

        # To aid localisation, we start off with an initial pose
        self._init_pose_subscriber = rospy.Subscriber("/initialpose",
                                                      PoseWithCovarianceStamped,
                                                      self._init_pose_callback)

        # Receive motion/odometry information
        self._odometry_subscriber = rospy.Subscriber("/odometry/filtered",
                                                     Odometry,
                                                     self._odometry_callback,
                                                     queue_size=1)


    def _init_pose_callback(self, pose):
        """ Called when RViz sends a user supplied initial pose estimate.

        Args:
            pose: The initial pose received

        """
        self._particle_filter.set_initial_pose(pose)

        last_pose = self._particle_filter.estimatedpose
        self._last_published_pose = deepcopy(last_pose)

        self._init_pose_received = True
        # Publish initial particle cloud
        self._cloud_publisher.publish(self._particle_filter.particlecloud)


    def _odometry_callback(self, odometry):
        """ Called when an odometry message is received.
        If the filter is initialised then execute a filter predict step with
        odometry followed by an update step using the latest laser.

        Args:
            odometry: The odometry message
        """
        if self._last_odometry == None:
            self._last_odometry = odometry
        self._latest_odometry = odometry
        if self._init_pose_received:
            estimated_pose = self._particle_filter.estimatedpose
            # Want to check the robot has moved enough to warrant an update
            if self._sufficientMovementDetected(estimated_pose):
                self._last_odometry = self._latest_odometry
                # Computation based on the motion model
                self._particle_filter.predict_from_odometry(odometry)
                # Update based on laser scan/sensor model
                self._particle_filter.update_filter(self._latest_scan)


    def _laser_callback(self, scan):
        """ Called when a new laser scan is received.
        Store a ref to the latest scan. If robot has moved enough,
        republish the latest pose to update RViz.

        Args:
            scan: The laser scan
        """
        self._latest_scan = scan
        if self._init_pose_received:

            # Publish the new pose
            current_estimate = self._particle_filter.estimatedpose
            self._amcl_pose_publisher.publish(current_estimate)
            estimatedpose =  PoseStamped()
            estimatedpose.pose = current_estimate.pose.pose
            estimatedpose.header.frame_id = "/map"
            self._pose_publisher.publish(estimatedpose)

            # Update record of previously-published pose
            self._last_published_pose = deepcopy(current_estimate)

            # Get updated particle cloud and publish it
            self._cloud_publisher.publish(self._particle_filter.particlecloud)

            # Get updated transform and publish it
            self._tf_publisher.publish(self._particle_filter.tf_message)


    def _sufficientMovementDetected(self, latest_pose):
        """ Compares the last published pose to the current pose.

        Args:
            latest_pose: The latest pose to be compared

        Returns:
            sufficient: True if movement > self._PUBLISH_DELTA, otherwise False
        """
        latest_pose = self._latest_odometry
        # Check that minimum required amount of movement
        # has occurred before re-publishing
        latest_x = latest_pose.pose.pose.position.x
        latest_y = latest_pose.pose.pose.position.y
        prev_x = self._last_odometry.pose.pose.position.x
        prev_y = self._last_odometry.pose.pose.position.y
        location_delta = abs(latest_x - prev_x) + abs(latest_y - prev_y)

        # Also check for difference in orientation: Take a zero-quaternion,
        # rotate forward by latest_rot, and rotate back by prev_rot,
        # to get difference)
        latest_rot = latest_pose.pose.pose.orientation
        prev_rot = self._last_odometry.pose.pose.orientation

        q = rotateQuaternion(Quaternion(w=1.0),
                             getHeading(latest_rot))   # Rotate forward
        q = rotateQuaternion(q, -getHeading(prev_rot)) # Rotate backward
        heading_delta = abs(getHeading(q))

        return (location_delta > self._PUBLISH_DELTA or
                heading_delta > self._PUBLISH_DELTA)

if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("pf_localisation")
    node = ParticleFilterLocalisationNode()
    rospy.spin()
