#! /usr/bin/env python3
import time
import cv2
import numpy as np
from PyQt5 import QtWidgets, uic
import sys
from PyQt5 import QtCore
import functions
import rospy
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from ur_msgs.srv import SetIO
import os
from pypylon import pylon
from numpy import ndarray, size

x_coef = 1 - 0.0182
x_offset = (-0.0658) / 10  # en cm
y_coef = 1 + 0.01835
y_offset = (-0.35) / 10  # en cm
z_offset = 0.5
board_vector = [-0.64961, -0.15675, -0.45695, 0.00086, 0.00434, 0.00028]

lines_coef = np.load(f'{os.getcwd()}/CalibrationRobot/up_and_down_img_folder/lines_coef.npy')


class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()
        self.imageDeBase = None
        uic.loadUi('ui/main.ui', self)
        self.PIN_CAM_DEVRACAGE = 5
        self.ON, self.OFF = 1, 0
        self.set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        self.robot_interface = functions.MoveGroupPythonInterface()
        self.robot_interface.move_group.set_pose_reference_frame("base")

        self.take_image.clicked.connect(self.show_image)
        self.quit_button.clicked.connect(QApplication.instance().quit)
        self.showMaximized()
        self.filename = None
        self.show()

    def camera_basler(self):

        tlFactory = pylon.TlFactory.GetInstance()
        devices = tlFactory.EnumerateDevices()
        print(devices)
        camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[1]))
        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        if camera.IsGrabbing():
            print(self.PIN_CAM_DEVRACAGE)
            self.set_io_interface(1, self.PIN_CAM_DEVRACAGE, self.ON)

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
        self.image.resize(1385, 1000)

        ratio, width, height = self.compute_ratio()

        image = cv2.imread(img_name)
        image = cv2.resize(image, (int(ratio * width), int(ratio * height)))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channels = image.shape
        step = channels * width
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        self.image.setFixedSize(1385, 1000)

        self.image.setPixmap(QPixmap.fromImage(qImg))
        self.image.setAlignment(QtCore.Qt.AlignCenter)

        self.image.mousePressEvent = self.getPos

    def compute_ratio(self):

        # Variables
        # width = 3840  # 3840
        # height = 2748  # 2748

        # W = 1385
        # H = 1000
        W = self.image.width()
        H = self.image.height()
        self.imageDeBase = cv2.imread(self.filename)

        width = size(self.imageDeBase, 1)
        height = size(self.imageDeBase, 0)

        ratio = min(W / width, H / height)
        print(W, H, width, height)
        return ratio, width, height

    def getPos(self, event):
        x = event.pos().x()
        y = event.pos().y()
        ratio, width, height = self.compute_ratio()
        print(ratio)
        print("coordinates in IHM picture: ", x, y)
        realX = int(x / ratio)
        realY = int(y / ratio)
        print("coordinates in Real Cemera size : ", realX, realY)

        start_pt, vect = self.compute_trajectory(realX, realY, 40)
        print(start_pt, vect)
        print('start : ', start_pt)
        print('vect : ', vect)
        position, vecteur, robot_command = self.mise_en_forme_commande_vecteur(start_pt, vect)
        print(position, vecteur)

        self.go_to_position(position, vecteur, robot_command)

    def go_to_position(self, position, vector, robot_command):
        print(position, vector, robot_command)
        self.robot_interface.go_to_pose_goal(robot_command)

    def mise_en_forme_commande_vecteur(self, pos_0, vect):

        print(pos_0)
        print(vect)

        commande = f"({pos_0[0][0] * 0.01},{pos_0[1][0] * 0.01},{(pos_0[2][0]) * 0.01},3.14,0,0)"
        vecteur = f"({vect[0][0]},{vect[1][0]},{vect[2][0]})"


        print(f"COMMANDE : {commande}")
        print(f"VECTEUR : {vecteur}")

        data_1 = pos_0[0][0] * 0.01 + (0.0519 * pos_0[0][0] * 0.01 + 0.0324) + 0.002
        data_2 = pos_0[1][0] * 0.01 + (0.0583 * pos_0[0][0] * 0.01 + 0.0036) + 0.0015

        #
        # robot_command = [pos_0[0][0] * 0.01 + 0.1, pos_0[1][0] * 0.01, pos_0[2][0] * 0.01, 3.14, 0, 0]
        #

        robot_command = [data_1, data_2, pos_0[2][0] * 0.01, 3.14, 0, 0]

        return commande, vecteur, robot_command

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
