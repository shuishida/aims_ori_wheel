#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


def get_collision_callback(view_range=0.5, thresh=0.15, ignore_thresh=0.05):
    def callback(msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        ranges = ranges[(-view_range <= angles <= view_range]
        ranges = ranges[ranges >= ignore_thresh]
        is_obst = ranges.min() <= thresh
        rospy.loginfo(f"{is_obst}")
    return callback


if __name__ == "__main__":
    rospy.init_node('laser_listener')
    rospy.Subscriber('/front/scan', LaserScan, get_collision_callback())
    rospy.spin()
