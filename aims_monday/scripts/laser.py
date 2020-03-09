#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np


class ObstacleDetector:

    def __init__(self):
        self.pub = rospy.Publisher('obstacle_detected', Bool, queue_size=1)
        self.scan = rospy.Subscriber('/front/scan', LaserScan, self.get_collision_callback(), queue_size=1)

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


if __name__ == "__main__":
    rospy.init_node('laser_listener')
    ObstacleDetector()
    rospy.spin()
