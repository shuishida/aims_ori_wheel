#!/usr/bin/env python
import numpy as np
import rospy


def select_ranges(ranges, angle_min, angle_max, view_range=1):
    """
    return: ranges: list, angles: list
    """
    ranges = np.array(ranges)
    angles = np.linspace(angle_min, angle_max, len(ranges))
    filter = np.where(angles >= -view_range, angles <= view_range, False)
    ranges = ranges[filter]
    angles = angles[filter]
    return ranges, angles


def get_collision_callback(self, view_range=1, thresh=0.60, ignore_thresh=0.05):
    def callback(msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        ranges = ranges[np.where(angles >= -view_range, angles <= view_range, False)]
        ranges = ranges[ranges >= ignore_thresh]
        is_obst = ranges.min() <= thresh
        self.pub.publish(is_obst)
        rospy.loginfo("Collision: %s, %s" % (is_obst, ranges[::10]))

    return callback


def get_grads(ranges, angle_min, angle_max, k, thresh, ignore_thresh=0.005):
    ranges, angles = select_ranges(ranges, angle_min, angle_max, view_range=1)

    force = np.zeros((2,))
    for r, t in zip(ranges, angles):
        if r >= ignore_thresh:
            u = np.array(np.cos(t), np.sin(t))
            f = k * (1 / r - 1 / thresh) * u / (r ** 2) if r < thresh else np.zeros((2,))
            force += f
    return force
