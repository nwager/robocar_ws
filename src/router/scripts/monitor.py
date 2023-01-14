#!/usr/bin/env python

import rospy
import serial
import struct
import time
import sys

router = serial.Serial('/dev/ttyACM0', 115200)

def main():
    rospy.init_node('router_monitor')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        start_write = time.time()
        router.write(struct.pack('2B', 0x01, 8))
        end_write = time.time()
        vel, steer = struct.unpack('<2f', router.read(8))
        end_read = time.time()
        rospy.loginfo("vel: %f, steer: %f", vel, steer)
        # rospy.loginfo("Took %f s to write, %f s to read", end_write - start_write, end_read - end_write)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

