#! /usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates

from math import pi, tau, dist, fabs, cos
import time

# Import des fonctions provenant du modèle tuto_python.py
import functions
from auto_mode_ur import auto_mode_ur

PIN_VENTURI_VIDE = 0
PIN_CAM_ORIENTATION = 4
PIN_CAM_DEVRACAGE = 5
ON, OFF = 1, 0

dictionnaire_joints = {'pos_vol': [-38.46, -79.78, 80.27, -90.46, -89.70, 85.17],
                       'pos_camera': [-74.08, -59.14, 82.87, -204.37, -16.88, 124.22],
                       'pos_goulotte': [-72.68, -42.4, 88.63, -226.64, -18.4, 123.99, ],
                       'position_prehensor': [-13.51, -48.22, 71.44, -113.19, -89.70, 110.25],
                       'position_prehensor_out': [-11.25, -18.03, 13.83, -85.76, -89.66, 110.25],
                       'pickup_position': [-15.09, -53.71, 130.91, -167.16, -89.90, 108.84],
                       }

dictionnaire_cartesian = {'pos_vol': [-0.729, 0.357, 0.222, 3.14, 0, 0],
                          'pos_camera_in_base': [-0.949, 0.763, 0.593, 1.893, -0.586, -1.896],
                          'pos_camera_in_camera_goulotte': [-0.2291, -0.1894, 0.5399, 1.9, -0.591, -1.885],
                          'pos_goulotte_in_base': [-0.970, 0.763, 0.298, 1.893, -0.586, -1.896],
                          'pos_goulotte_in_camera_goulotte': [-0.2777, -0.1906, 0.2443, 1.9, -0.591, -1.885]}


def main():
    try:

        robot_interface = functions.MoveGroupPythonInterface()
        set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)

        gripperPublisher = rospy.Publisher('chatter', String, queue_size=10)
        rate = rospy.Rate(10)  # 10hz

        while True:

            print("")
            print("==========================================================")
            print("Demonstration program for robot displacement ")
            print("==========================================================")
            print("0 ==== Auto Mode ")
            print("1 ==== Go to init position ")
            print("2 ==== Go to gripper position ")
            print("3 ==== Go to pickup position ")
            print("4 ==== Start Venturi ")
            print("5 ==== Stop Venturi ")
            print("6 ==== Open Gripper ")
            print("7 ==== Close Gripper ")
            user_input = input("Enter a number (or 'q' to quit): ")

            if user_input == 'q':
                break
            choice = int(user_input)
            if choice == 0:
                init_position = dictionnaire_joints['pos_vol']
                prehensor_position = dictionnaire_joints['position_prehensor']
                prehensor_position_out = dictionnaire_joints['position_prehensor_out']
                pickup_position = dictionnaire_joints['pickup_position']
                camera_position = dictionnaire_joints['pos_camera']
                box_position = dictionnaire_joints['pos_goulotte']
                auto_mode_ur(robot_interface, set_io_interface, init_position, prehensor_position,
                             prehensor_position_out, pickup_position, camera_position, box_position)

            elif choice == 1:
                print("============ init position ============")
                robot_interface.go_to_joint_state(functions.convert_deg_to_rad(dictionnaire_joints['pos_vol']))

            elif choice == 2:
                print("============ gripper position ============")
                robot_interface.go_to_joint_state(
                    functions.convert_deg_to_rad(dictionnaire_joints['position_prehensor']))
                robot_interface.go_to_joint_state(
                    functions.convert_deg_to_rad(dictionnaire_joints['position_prehensor_out']))

            elif choice == 3:
                print("============ pickup position ============")
                robot_interface.go_to_joint_state(functions.convert_deg_to_rad(dictionnaire_joints['pickup_position']))

            elif choice == 4:
                print("============ start Venturi ============")
                set_io_interface(1, PIN_VENTURI_VIDE, ON)
            elif choice == 5:
                print("============ stop Venturi ============")
                set_io_interface(1, PIN_VENTURI_VIDE, OFF)
            elif choice == 6:
                print("============ open gripper ============")
                str_choice = "open"
                rospy.loginfo(str_choice)
                gripperPublisher.publish(str_choice)
                rate.sleep()
            else:
                print("============ close gripper ============")
                str_choice = "close"
                rospy.loginfo(str_choice)
                gripperPublisher.publish(str_choice)
                rate.sleep()

        #
        # ## Venturi ON
        # # """
        # set_io_interface(1,PIN_VENTURI_VIDE,ON)
        # """
        #
        # ## Follow vecteur
        # input("============ Presser `Enter` pour commencer la descente")
        #
        # vecteur = [-0.3, -0.3, -0.1]
        #
        # pos_goulotte = robot_interface.move_group.get_current_pose()
        # print((pos_goulotte))
        # pos_goulotte.pose.position.x += vecteur[0]
        # pos_goulotte.pose.position.y += vecteur[1]
        # pos_goulotte.pose.position.z += vecteur[2]
        # # robot_interface.move_group.set_pose_target(pos_goulotte)
        # print((pos_goulotte))
        # ##robot_interface.prehension(pos_goulotte)
        # # success = robot_interface.move_group.go(wait=True)
        # # robot_interface.move_group.stop()
        # # robot_interface.move_group.clear_pose_targets()
        #
        # # success = robot_interface.move_group.go(wait=True)
        # # robot_interface.move_group.stop()
        # # robot_interface.move_group.clear_pose_targets()
        #
        # input("============ FIN TEST")
        #
        # ## Until capteur effort - vide
        #
        # ## Go to position 0
        #
        # ## Go to position vol
        #
        # ## Camera dévracage ON - OFF
        # """
        # set_io_interface(1,PIN_CAM_DEVRACAGE,ON)
        # time.sleep(0.5)
        # set_io_interface(1,PIN_CAM_DEVRACAGE,OFF)
        # """
        #
        # ## Go to position caméra
        # input("============ Presser `Enter` pour placer le robot à pos_camera")
        # robot_interface.go_to_joint_state(functions.convert_deg_to_rad(dictionnaire_joints['pos_camera']))
        #
        # ## Camera orientation ON - OFF
        # """
        # set_io_interface(1,PIN_CAM_ORIENTATION,ON)
        # time.sleep(0.5)
        # set_io_interface(1,PIN_CAM_ORIENTATION,OFF)
        # """
        #
        # ## Go to position rotation
        # input("============ Presser `Enter` pour placer le robot à pos_rot")
        # angle_correction_deg = -90
        # angle_correction_rad = angle_correction_deg * pi / 180
        # joint_goal = robot_interface.move_group.get_current_joint_values()
        # joint_goal[5] += angle_correction_rad
        # robot_interface.move_group.go(joint_goal, wait=True)
        # robot_interface.move_group.stop()
        #
        # ## Go down to goulotte
        # input("============ Presser `Enter` pour placer le robot à pos_goulotte")
        # pos_goulotte = robot_interface.move_group.get_current_pose()
        # pos_goulotte.pose.position.z -= 0.4
        # robot_interface.move_group.set_pose_target(pos_goulotte)
        # success = robot_interface.move_group.go(wait=True)
        # robot_interface.move_group.stop()
        # robot_interface.move_group.clear_pose_targets()
        #
        # ## Venturi OFF
        # """
        # set_io_interface(1,PIN_VENTURI_VIDE,OFF)
        # """
        #
        # ## Go to position goulotte
        # input("============ Presser `Enter` pour placer le robot à pos_camera")
        # robot_interface.go_to_joint_state(functions.convert_deg_to_rad(dictionnaire_joints['pos_goulotte']))
        #
        # ## Go to position vol
        # input("============ Presser `Enter` pour placer le robot à pos_vol")
        # robot_interface.go_to_joint_state(functions.convert_deg_to_rad(dictionnaire_joints['pos_vol']))


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    # to start the program
    main()
