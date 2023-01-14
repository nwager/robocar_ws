#!/usr/bin/env python

from __future__ import print_function

import rospy
import serial
import struct
import time

from router.srv import SetControl, SetControlResponse

ACK_VALUE = 0x00
NACK_VALUE = 0xFF

CMD_GET_ACCEL = 0x00
CMD_GET_CONTROL = 0x01
CMD_CALIBRATE_ACCEL = 0x02
CMD_RESET = 0x80
CMD_SET_CONTROL = 0x81
CMD_SET_ACCEL_OFFSET = 0x82

CONTROL_SIZE = struct.calcsize('2f')

def handle_set_control(req):
    msg = struct.pack('<2B2f', CMD_SET_CONTROL, CONTROL_SIZE, req.vel, req.steer)
    router.write(msg)
    ack = get_router_ack()

    router.write(struct.pack('2B', CMD_GET_CONTROL, CONTROL_SIZE))
    vel, steer = struct.unpack('<2f', router.read(CONTROL_SIZE))
    print("GetControls - vel: %f, steer: %f" % (vel, steer))

    return SetControlResponse(0 if ack else 1)

def set_control_server():
    rospy.init_node('set_control_server')

    global router
    router = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    if attempt_reset_router():
        rospy.loginfo("Successfully reset router device.")
    else:
        raise RuntimeError("Failed to reset router device.")

    s = rospy.Service('set_control', SetControl, handle_set_control)
    rospy.spin()

def attempt_reset_router(timeout=5):
    start_time = time.time()
    did_reset = False
    while not rospy.is_shutdown() and not did_reset:
        if time.time() - start_time > timeout:
            break
        router.write(struct.pack('2B', CMD_RESET, 0))
        did_reset = get_router_ack()
        time.sleep(0.2)

    return did_reset

def get_router_ack(timeout=5):
    res = ''
    i = 0
    while len(res) != 1:
        if i >= timeout / router.timeout:
            raise RuntimeError("Router failed to respond")
        res = router.read(1)
        i += 1
    res, = struct.unpack('B', res)
    return res == ACK_VALUE

if __name__ == "__main__":
    set_control_server()
