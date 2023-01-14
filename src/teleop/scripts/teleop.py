#!/usr/bin/env python

from game_controller import GameController
import rospkg
import rospy
from robocar_common import topics
from robocar_common.parameters import MAX_STEER_ANGLE, MIN_STEER_ANGLE
from robocar_common.msg import RobocarControl
from router.srv import SetControl

motor_enabled = False

def main():
    rospy.init_node('teleop_node')
    pub = rospy.Publisher(topics.CONTROL_ACTION, RobocarControl, queue_size=1)
    rospack = rospkg.RosPack()

    controller = GameController(
        "Xbox Wireless Controller",
        rospack.get_path('teleop') + "/config/xbox_code_map.yaml",
        joystick_min=-1.0, joystick_max=1.0,
        trigger_min=0.0, trigger_max=1.0,
        event_filter=['a'],
        filtered_event_cb=motor_toggle_cb
    )
    controller.begin()

    rospy.wait_for_service('set_control')

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        throttle = controller.trigger_right
        reverse_throttle = controller.trigger_left
        steer = controller.joystick_x_left

        vel = 2.0 * (throttle - reverse_throttle) if motor_enabled else 0.0
        if throttle != 0 and not motor_enabled:
            rospy.loginfo("Motor is disabled")
        # flip steer direction
        steer_angle = (MIN_STEER_ANGLE - MAX_STEER_ANGLE) * steer / 2
        # msg = RobocarControl(speed=speed, steering_angle=steer_angle)
        # pub.publish(msg)

        try:
            set_control = rospy.ServiceProxy('set_control', SetControl)
            res = set_control(vel, steer_angle)
            if res.status != 0:
                rospy.logerr("Received nonzero response: %d" % res.status)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        rate.sleep()

def motor_toggle_cb(controller, name):
    global motor_enabled
    if controller[name] == 1:
        motor_enabled = not motor_enabled

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
