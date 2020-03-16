#!/usr/bin/env python

import sys
import rospy
from aims_jackal_sim.srv import *
import numpy as np

if __name__ == "__main__":
    rospy.wait_for_service('sim_reset')
    srv = rospy.ServiceProxy('sim_reset', SimReset)
    resp = srv(2)

    if resp.result:
        print('Sim reset successfully.')
    else:
        print('Failed to reset sim.')
