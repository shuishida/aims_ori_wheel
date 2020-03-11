#! /usr/bin/env python
import rospy
import actionlib
from aims_msgs.msg import *
from geometry_msgs.msg import PoseStamped

def search_room_client():
    client = actionlib.SimpleActionClient('search_room', SearchRoomAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    client.send_goal(SearchRoomGoal())
    client.wait_for_result()
    print(client.get_result())

if __name__ == '__main__':
    rospy.init_node('test_search_room')
    res = search_room_client()
    rospy.spin()
