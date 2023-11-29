#!/usr/bin/python3

from plateform.robot.specific.ur.connexion.robot_state import set_robot_state
from plateform.robot.specific.ur.connexion.get_robot_mode import get_robot_mode
from plateform.robot.specific.ur.connexion.reverse_connexion import connexion_state
from plateform.robot.specific.ur.plateform_class import Platform


def robot_connexion(state: bool):
    """This function allowed you to established the robot connexion

    :param state: boolean state, no default value
    :type state: bool

    :return: success, message
    :rtype: bool, str


    """
    if state:
        print("======Start Robot Connexion======")
        success, message = set_robot_state(True)
        if success:
            while True:
                robot_state = get_robot_mode()
                if robot_state == 7:
                    print("=========Robot mode is {} ======== ".format(robot_state))
                    print("=========Start reverse connexion with the command interface===========")

                    success, message, robot = connexion_state(True)
                    if success:
                        print("=========You are actually connected===========")
                        Platform().storeInterface(robot)
                        return success, message, robot
                    else:
                        return success, message, robot
        else:
            return success, message, None
    else:
        print("========Start Stoping brakes please wait ======")
        success, message = set_robot_state(False)
        if success:
            while True:
                robot_state = get_robot_mode()
                if robot_state == 3:
                    print("=========Robot mode is {} ======== ".format(robot_state))
                    print("=========Stop reverse connexion with the command interface===========")
                    success, message, robot = connexion_state(False)
                    if success:
                        print("=========You are actually disconnected===========")

                        return success, message
                    else:
                        return success, message

        else:
            return success, message


if __name__ == '__main__':
    success, message = robot_connexion(state=False)
    print(success, message)
