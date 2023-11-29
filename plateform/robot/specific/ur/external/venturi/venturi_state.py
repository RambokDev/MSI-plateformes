#!/usr/bin/python3

import rospy
from ur_msgs.srv import SetIO

PIN_VENTURI_VIDE = 0


def venturi_state(state):
    set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    print("=========Venturi {}=======".format(state))
    set_io_interface(1, PIN_VENTURI_VIDE, state)


if __name__ == '__main__':
    ON, OFF = 1, 0
    venturi_state(1, ON)
