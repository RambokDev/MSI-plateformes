#!/usr/bin/python3

from robot.ur.commands.robot_state import set_robot_state
from robot.ur.commands.get_robot_mode import get_robot_mode
from robot.ur.commands.reverse_connexion import connexion_state


def robot_connexion(state: bool):
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
                        return success, message, robot
                    else:
                        return success, message, robot
        else:
            return success, message
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
