#!/usr/bin/env python
""" File contains utility functions for visualisation. 
Documentation can be found at:
http://wiki.ros.org/rviz/DisplayTypes/Marker

Owner: Charlie Street
"""

import rospy
import roslib
import subprocess
import math

# messages
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point



def is_simulated():
    """ Check if stage simulation is being used. """
    p = subprocess.Popen(['ps', '-ax'], stdout=subprocess.PIPE)
    stdout, _ = p.communicate()
    if 'stage_ros/stageros' in stdout:
        return True

def pol_to_cart(pol):
    """ Convert polar coordinate to cartesian coordinate. """
    r = pol[0]
    theta = pol[1]
    return (r*math.cos(theta), r*math.sin(theta))


def log(x):
    """ Wrapper around rospy.loginfo. """
    rospy.loginfo(x)

# Set global variables depending on whether in simulation or not
if is_simulated():
    robot_ref_frame = "/base_laser_link"
    #world_ref_frame = "/odom"
    world_ref_frame = "/map"
else:
    robot_ref_frame = "/odometry/filtered"
    #world_ref_frame = "/odom"
    world_ref_frame = "/map"


class Viz(object):
    """ Class for visualisation functions. """
    def __init__(self):
        self.mark_pub = rospy.Publisher('visualization_marker', 
                                        Marker, \
                                        queue_size=10)
        self.id_counter = 0

        self.robot_ref_frame = robot_ref_frame
        self.world_ref_frame = world_ref_frame



    def place_marker(self, x, y, size=0.1, mtype=Marker.SPHERE,
                     color=(0, 1, 0, 1), duration=None, 
                     reference_frame=world_ref_frame):
        """ Place an arbitrary marker in Rviz for some duration. 
        
        Args:
            x: The x position
            y: The y poisition
            mtype: The marker type
            color: The colour of the marker
            duration: How long should the marker appear for
            reference_frame: Reference for position of marker
        """
        m = Marker()

        if is_simulated() and reference_frame==robot_ref_frame:
            x, y = y, -x

        # specify reference frame (options in RViz Global Options > Fixed Frame)
        # markers relative to 0,0 in odometry
        m.header.frame_id = reference_frame
        m.header.stamp    = rospy.Time.now()

        # marker with same namespace and id overrides existing
        m.ns = "my_markers"
        m.id = self.id_counter
        self.id_counter += 1

        m.type = mtype
        m.action = m.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        m.scale.x = size
        m.scale.y = size
        m.scale.z = size
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]

        if duration is None:
            m.lifetime = rospy.Duration() # forever
        else:
            m.lifetime = rospy.Duration(duration)

        #log('placing marker:\n' + str(m.pose.position))
        self.mark_pub.publish(m)


    def place_robot_marker(self, x, y, size=0.1, color=(0, 1, 0, 1)):
        """ Place a marker for the robot (?). 
        
        Args:
            x: The x position
            y: The y poisition
            size: How big should the marker be?
            color: The colour of the marker

        """
        self.place_marker(x, y, size, color=color, 
                          reference_frame=robot_ref_frame)


    def place_rect(self, x, y, w, h, reference_frame=world_ref_frame,
                   width=0.05, color=(0, 0, 1, 1), duration=None):
        """ Place a rectangle in Rviz. 
        
        Args:
            x: The x position
            y: The y poisition
            w: The rectangle width
            h: The rectangle height
            reference_frame: Reference for marker position
            width: ?
            color: The colour of the marker
            duration: How long should the marker appear for?
        """
        m = Marker()

        # specify reference frame (options in RViz Global Options > Fixed Frame)
        # markers relative to 0,0 in odometry
        m.header.frame_id = reference_frame
        m.header.stamp    = rospy.Time.now()

        # marker with same namespace and id overrides existing
        m.ns = "my_markers"
        m.id = self.id_counter
        self.id_counter += 1

        m.type = m.LINE_STRIP
        m.action = m.ADD

        m.points = [Point(x, y, 0), Point(x, y+h, 0), Point(x+w, y+h, 0),
                    Point(x+w, y, 0), Point(x, y, 0)]

        m.scale.x = width
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]

        if duration is None:
            m.lifetime = rospy.Duration() # forever
        else:
            m.lifetime = rospy.Duration(duration)

        #log('placing rect:\n' + str(m.pose.position))
        self.mark_pub.publish(m)


    def clear_markers(self):
        """ Clear all markers in RViz. """
        m = Marker()

        # specify reference frame (options in RViz Global Options > Fixed Frame)
        # markers relative to 0,0 in odometry
        m.header.frame_id = world_ref_frame
        m.header.stamp    = rospy.Time.now()

        # marker with same namespace and id overrides existing
        m.ns = "my_markers"
        m.id = self.id_counter
        self.id_counter += 1

        m.action = 3 # delete all

        self.mark_pub.publish(m)


    def draw_AABB(self, aabb, color=(0, 0, 1, 1)):
        """ Draw an AABB box (i.e. a rectangle).
        
        Args:
            aabb: The aabb box
            colour: The box colour

        """
        # need to flip x and y if simulated
        if is_simulated():
            self.place_rect(aabb.y, -aabb.x, aabb.h, -aabb.w,
                            reference_frame=robot_ref_frame, color=color)
        else:
            pass # not implemented


    def draw_lines(self, points, size=0.1, color=(0, 1, 0, 1), duration=None,
            reference_frame=world_ref_frame):
        """ Draw lines in RViz.
        pass lines as [(x,y), (x_2, y_2)] to draw lines between every adjacent
        pair in the list.

        Args:
            points: List of points
            size: Size of lines
            color: Colour of lines
            duration: How long should lines appear for
            reference_frame: Reference for line position
        """
        
        m = Marker()

        if is_simulated() and reference_frame==robot_ref_frame:
            x, y = y, -x

        # specify reference frame (options in RViz Global Options > Fixed Frame)
        # markers relative to 0,0 in odometry
        m.header.frame_id = reference_frame
        m.header.stamp    = rospy.Time.now()

        # marker with same namespace and id overrides existing
        m.ns = "my_markers"
        m.id = self.id_counter
        self.id_counter += 1

        m.type = m.LINE_LIST
        m.action = m.ADD

        m.scale.x = size

        m.points = [Point(x,y,0) for x,y in points]

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = color[3]

        if duration is None:
            m.lifetime = rospy.Duration() # forever
        else:
            m.lifetime = rospy.Duration(duration)

        #log('placing marker:\n' + str(m.pose.position))
        self.mark_pub.publish(m)





