#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3

if __name__ == "__main__":
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rospy.init_node('driver')
	r = rospy.Rate(10) 

	fwd = Twist(linear=Vector3(x=0.1))
	turn = Twist(angular=Vector3(z=0.1))


	# simple time-based trigger
	now = rospy.get_rostime()
	then = now + rospy.Duration.from_sec(4)

	while not rospy.is_shutdown() and rospy.get_rostime() < then:
	   pub.publish(turn)
	   r.sleep()
