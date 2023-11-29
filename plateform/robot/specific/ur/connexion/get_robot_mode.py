#!/usr/bin/python3
import time

import rospy
from ur_dashboard_msgs.srv import GetRobotMode
import threading
from time import sleep
from threading import Thread


def get_robot_mode_ros():
    rospy.wait_for_service('/ur_hardware_interface/dashboard/get_robot_mode')
    robot_get_mode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode',
                                        GetRobotMode)
    try:
        resp = robot_get_mode()
        if resp.success:
            return resp.robot_mode.mode
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))




class GetRobotModeThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.robot_mode = None

    def run(self):
        self.robot_mode = get_robot_mode_ros()




def get_robot_mode():
    """
    This function allowed you to get the current robot mode
    """
    # robot_mode = None
    # robot_mode_thread = threading.Thread(target=get_robot_mode_thread, args=(robot_mode,))
    # robot_mode_thread.start()
    # return robot_mode

    # create a new thread
    thread = GetRobotModeThread()
    # start the thread
    thread.start()
    # wait for the thread to finish
    thread.join()
    # get the value returned from the thread
    data = thread.robot_mode
    return data







if __name__ == '__main__':
    # rospy.init_node("test_robotUR")
    #
    mode = get_robot_mode()
    print(mode)
