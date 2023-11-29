#!/usr/bin/python3

import rospy
from ur_msgs.srv import SetIO

PIN_CAM_DEVRACAGE = 5
PIN_CAM_ORIENTATION = 4


def camera_lights_state(camera_type, state):
    set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)

    if camera_type == 0:
        light = PIN_CAM_ORIENTATION
        print("Orientation Activation light")
        set_io_interface(1, light, state)
    else:
        light = PIN_CAM_DEVRACAGE
        print("Devracage Activation light")
        set_io_interface(1, light, state)


if __name__ == '__main__':
    ON, OFF = 1, 0
    camera_lights_state(1, ON)
