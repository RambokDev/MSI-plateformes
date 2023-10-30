#! /usr/bin/env python3
import time
import cv2
import numpy as np
from PyQt5 import QtWidgets, uic
import sys

import functions
import rospy
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from ur_msgs.srv import SetIO
import os
from pypylon import pylon
from numpy import ndarray, size

from RobotUR import RobotUR
import geometry_msgs.msg as geometry_msgs

x_coef = 1 - 0.0182
x_offset = (-0.0658) / 10  # en cm
y_coef = 1 + 0.01835
y_offset = (-0.35) / 10  # en cm
z_offset = 0.5
board_vector = [-0.64961, -0.15675, -0.45695, 0.00086, 0.00434, 0.00028]
# board_vector = [-0.66159,-0.08739,-0.46456,0.004,0.005,0.006]
lines_coef = np.load(f'{os.getcwd()}/CalibrationRobot/up_and_down_img_folder/lines_coef.npy')

i, j, image_circle, image_vierge = 0, 0, 0, 0


class Ui(QtWidgets.QMainWindow, ):
    def __init__(self):
        super(Ui, self).__init__()
        self.imageDeBase = None
        uic.loadUi('ui/main.ui', self)
        self.PIN_CAM_DEVRACAGE = 5
        self.ON, self.OFF = 1, 0
        self.set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)

        self.take_image.clicked.connect(self.show_image)
        self.quit_button.clicked.connect(QApplication.instance().quit)
        # self.setGeometry(0, 0, 1920, 1080)

        self.showMaximized()
        self.filename = None
        self.show()

    def camera_basler(self):

        tlFactory = pylon.TlFactory.GetInstance()
        devices = tlFactory.EnumerateDevices()
        print(devices)

        camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[0]))
        camera.Open()
        camera.Width = 3000
        camera.Height = 2000

        camera.CenterX.SetValue(True)
        camera.CenterY.SetValue(True)
        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        if camera.IsGrabbing():
            print(self.PIN_CAM_DEVRACAGE)
            self.set_io_interface(1, self.PIN_CAM_DEVRACAGE, self.OFF)

            grab = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            # grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_Return)
            if grab.GrabSucceeded():
                # image = converter.Convert(grab)
                # img = grab.GetArray()

                img = pylon.PylonImage()

                img.AttachGrabResultBuffer(grab)
                self.filename = "images/saved_pypylon_img.png"
                img.Save(pylon.ImageFileFormat_Png, self.filename)
                time.sleep(2)
                self.set_io_interface(1, self.PIN_CAM_DEVRACAGE, self.OFF)

                return self.filename

            camera.Close()

    def show_image(self):

        print("=====Display image =====")
        img_name = self.camera_basler()
        # self.image.resize(1385, 1000)
        self.image.setFixedSize(1385, 923)

        ratio, width, height = self.compute_ratio()

        image = cv2.imread(img_name)
        # image2 = imutils.resize(image, width=1385)

        image = cv2.resize(image, (round(ratio * width), round(ratio * height)))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channels = image.shape
        print(height, width, channels)
        step = channels * width
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        # self.image.setFixedSize(1385, 1000)
        self.image.setPixmap(QPixmap.fromImage(qImg))
        # self.image.setAlignment(QtCore.Qt.AlignCenter)

        self.image.mousePressEvent = self.getPos

    def compute_ratio(self):

        W = self.image.width()
        H = self.image.height()
        # H = 923
        self.imageDeBase = cv2.imread(self.filename)

        width = size(self.imageDeBase, 1)
        height = size(self.imageDeBase, 0)
        ratio = min(W / width, H / height)
        print(ratio, W, H, width, height)
        return ratio, width, height

    def getPos(self, event):
        x = event.pos().x()
        y = event.pos().y()
        ratio, width, height = self.compute_ratio()
        print(ratio)
        print("coordinates in IHM picture: ", x, y)
        realX = round(x / ratio)
        realY = round(y / ratio)
        print(realX, realY)
        # realX = 0
        # realY = 0
        # print("coordinates in Real Cemera size : ", realX2, realY2)

        start_pt, vect = self.compute_trajectory(realX, realY, 40)
        print(start_pt, vect)
        print('start : ', start_pt)
        print('vect : ', vect)
        robot_command, vector = self.mise_en_forme_commande_vecteur(start_pt, vect)
        self.go_to_position(robot_command, vector)

    def go_to_position(self, robot_command, vector):
        print(robot_command, vector)
        rospy.init_node("test_robotUR")
        myRobot = RobotUR()
        myRobot.go_to_initial_position(5)

        myRobot.go_to_pose(geometry_msgs.Pose(
            geometry_msgs.Vector3(robot_command[0], robot_command[1], robot_command[2]),
            RobotUR.tool_down_pose
        ))
        myRobot.relative_move(vector[0], vector[1], vector[2])

    def mise_en_forme_commande_vecteur(self, pos_0, vect):

        # commande = f"({pos_0[0][0] * 0.01},{pos_0[1][0] * 0.01},{(pos_0[2][0]) * 0.01},3.14,0,0)"
        # vecteur = f"({vect[0][0]},{vect[1][0]},{vect[2][0]})"
        #
        # print(f"COMMANDE : {commande}")
        # print(f"VECTEUR : {vecteur}")
        print(pos_0)
        vecteur = [vect[0][0] * 0.01, vect[1][0] * 0.01, vect[2][0] * 0.01]
        data_1 = (pos_0[0][0] * 0.01) + ((0.0519 * (pos_0[0][0] * 0.01)) + 0.0324) + 0.002
        data_2 = (pos_0[1][0] * 0.01) + ((0.0583 * (pos_0[1][0] * 0.01)) + 0.0036) + 0.0015

        # robot_command = [pos_0[0][0] * 0.01, pos_0[1][0] * 0.01, pos_0[2][0] * 0.01, 3.14, 0, 0]

        robot_command = [data_1, data_2, pos_0[2][0] * 0.01]

        return robot_command, vecteur

    def compute_trajectory(self, i, j, z0):

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
        # start_pt = np.array([[(start_pt[0][0] - 0.0066) * (1 - 0.01815)], [(start_pt[1][0] - 0.03505) * (1 + 0.01845)]])

        return start_pt, vect


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()


if __name__ == '__main__':
    main()
