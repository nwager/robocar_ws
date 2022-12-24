#!/usr/bin/env python

from __future__ import print_function
import rospy
import serial
import struct
import time
from robocar_common import topics
from robocar_common.msg import RobocarControl

ACK_VALUE = 0x00
CMD_RESET = 128
CMD_SET_CONTROL = 129

router = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def main():
    rospy.init_node('router_receiver')

    # reset procedure
    router.reset_output_buffer()
    router.reset_input_buffer()
    if not attempt_reset_router():
        raise RuntimeError("Failed to reset router device.")
    router.reset_input_buffer()

    rospy.Subscriber(topics.CONTROL_ACTION, RobocarControl, control_cb)

    rospy.spin()

def attempt_reset_router(timeout=5):
    start_time = time.time()
    did_reset = False
    while not did_reset and not rospy.is_shutdown():
        if time.time() - start_time > timeout:
            break
        send_command(CMD_RESET, 0)
        did_reset = get_router_ack()
        time.sleep(0.2)
    
    if did_reset:
        rospy.loginfo("Successfully reset router device.")
    else:
        rospy.loginfo("Failed to reset router device.")
    return did_reset

def control_cb(msg):
    speed, steer = msg.speed, msg.steering_angle
    cmd = CMD_SET_CONTROL
    payload_size = 8
    send_command(cmd, payload_size)
    router.write(struct.pack('2f', speed, steer))
    response = get_router_ack()
    

def send_command(command, payload_size):
    router.write(struct.pack('2B', command, payload_size))

def get_router_ack():
    ack = struct.unpack('B', router.read(1))[0]
    return ack == ACK_VALUE


if __name__ == "__main__":
    main()
