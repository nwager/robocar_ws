#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
import time

from router.srv import SetControl

def set_control_client(vel, steer):
    rospy.wait_for_service('set_control')
    try:
        set_control = rospy.ServiceProxy('set_control', SetControl)
        res = set_control(vel, steer)
        return res.status
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    vel, steer = [float(x) for x in sys.argv[1:3]]
    print("Requesting control ( vel: %f, steer: %f )" % (vel, steer))
    start = time.time()
    res = set_control_client(vel, steer)
    stop = time.time()
    print("Received response %d in %f ms" % (res, (stop-start)*1000))