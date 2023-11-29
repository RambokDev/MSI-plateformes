#!/usr/bin/python3
import os
import cv2
import numpy as np

x_coef = 1 - 0.0182
x_offset = (-0.0658) / 10  # en cm
y_coef = 1 + 0.01835
y_offset = (-0.35) / 10  # en cm
z_offset = 0.5
board_vector = [-0.64961, -0.15675, -0.45695, 0.00086, 0.00434, 0.00028]
lines_coef = np.load(f'{os.getcwd()}/ihm_tests/CalibrationRobot/up_and_down_img_folder/lines_coef.npy')
i, j, image_circle, image_vierge = 0, 0, 0, 0


def formatting_commands(pos_0, vect):
    """
    This function allowed you to format the commands vector and position
    :param pos_0: a list of integers representing the x,y,z
    :param vect: a list of integers representing the Rx,Ry,Rz
    """
    vecteur = [vect[0][0] * 0.01, vect[1][0] * 0.01, vect[2][0] * 0.01]
    data_1 = (pos_0[0][0] * 0.01) + ((0.0519 * (pos_0[0][0] * 0.01)) + 0.0324) + 0.002
    data_2 = (pos_0[1][0] * 0.01) + ((0.0583 * (pos_0[1][0] * 0.01)) + 0.0036) + 0.0015
    robot_command = [data_1, data_2, pos_0[2][0] * 0.01]
    return robot_command, vecteur


def compute_trajectory(i, j, z0):
    """
    This function compute the trajectory of the robot
    """
    r0 = lines_coef[j][i][0]
    d = lines_coef[j][i][1]
    end_t = 0
    start_t = (z0 - r0[2]) / d[2]

    end_pt = np.array(
        [[((r0[0] + d[0] * end_t) + x_offset) * x_coef], [((r0[1] + d[1] * end_t) + y_offset) * y_coef],
         [(r0[2] + d[2] * end_t) + z_offset]])

    start_pt = np.array(
        [[((r0[0] + d[0] * start_t) + x_offset) * x_coef], [((r0[1] + d[1] * start_t) + y_offset) * y_coef],
         [(r0[2] + d[2] * start_t) + z_offset]])

    board_rot, jac = cv2.Rodrigues(np.array([[board_vector[3]], [board_vector[4]], [board_vector[5]]]))
    board_trans = np.array([[board_vector[0] * 100], [board_vector[1] * 100], [board_vector[2] * 100]])

    end_pt = np.dot(board_rot, end_pt) + board_trans
    start_pt = np.dot(board_rot, start_pt) + board_trans

    vect = end_pt - start_pt

    return start_pt, vect
