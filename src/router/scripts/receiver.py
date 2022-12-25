#!/usr/bin/env python

from __future__ import print_function
import rospy
import serial
import struct
import time
from robocar_common import topics
from robocar_common.msg import RobocarControl

ACK_VALUE = 0x00

CMD_RESET = 0x80
CMD_CALIBRATE_ACCEL = 0x02
CMD_SET_CONTROL = 0x81
CMD_SET_ACCEL_OFFSET = 0x82

router = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def main():
    rospy.init_node('router_receiver')

    # reset procedure
    router.reset_output_buffer()
    router.reset_input_buffer()
    if not attempt_reset_router():
        raise RuntimeError("Failed to reset router device.")
    else:
        rospy.loginfo("Successfully reset router device.")
    
    send_command(CMD_SET_ACCEL_OFFSET, 2*struct.calcsize('3f'))
    send_data(
        '<6f',
        0.6390673518180847, -0.09235493093729019, 10.13368034362793, # grav
        0.9912459850311279, 0.06961558014154434, 0.11218287795782089 # fwd
    )

    # if not attempt_calibrate():
    #     raise RuntimeError("Failed to calibrate accelerometer.")
    # else:
    #     rospy.loginfo("Successfully calibrated accelerometer.")

    rospy.Subscriber(topics.CONTROL_ACTION, RobocarControl, control_cb)

    rospy.spin()

def attempt_reset_router(timeout=5):
    start_time = time.time()
    did_reset = False
    while not rospy.is_shutdown() and not did_reset:
        if time.time() - start_time > timeout:
            break
        send_command(CMD_RESET, 0)
        did_reset = get_router_ack()
        time.sleep(0.2)

    return did_reset

def attempt_calibrate(timeout=3):
    start_time = time.time()
    rospy.loginfo("Attempting to calibrate accelerometer...")
    send_command(CMD_CALIBRATE_ACCEL, 0)
    while not rospy.is_shutdown() and time.time() < start_time + timeout:
        did_calibrate = get_router_ack()
    return did_calibrate

def control_cb(msg):
    send_control(msg.speed, msg.steering_angle)

def send_control(speed, steer):
    cmd = CMD_SET_CONTROL
    payload_size = 8
    send_command(cmd, payload_size)
    router.write(struct.pack('2f', speed, steer))

def send_command(command, payload_size):
    router.write(struct.pack('<2B', command, payload_size))

def send_data(fmt, *data):
    router.write(struct.pack(fmt, *data))

def get_router_ack():
    ack_read = router.read(1)
    if len(ack_read) != 1:
        return False
    ack = struct.unpack('B', ack_read)[0]
    return ack == ACK_VALUE


if __name__ == "__main__":
    try:
        main()
    finally:
        send_control(0, 0)
        router.close()
