from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QPixmap
import sys
import numpy as np
import cv2

import os

from PySide2.QtCore import Slot
from pypylon import pylon
from PyQt5.QtGui import QImage, QPixmap
from PIL import Image
import PIL
x_coef = 1 - 0.0182
x_offset = (-0.0658) / 10  # en cm
y_coef = 1 + 0.01835
y_offset = (-0.35) / 10  # en cm
z_offset = 0.5
board_vector = [-0.64961, -0.15675, -0.45695, 0.00086, 0.00434, 0.00028]

lines_coef = np.load(f'{os.getcwd()}/CalibrationRobot/up_and_down_img_folder/lines_coef.npy')


class MainWidget(QWidget):

    def __init__(self):
        super().__init__()
        self.button_show()

    def button_show(self):
        button = QPushButton('Click Me ', self)
        button.resize(200, 60)
        button.click.connect(self.on_click)

    def on_click(self):
        print("usr click")


class ImageWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.show_image()




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

            grab = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            # grab = camera.RetrieveResult(2000, pylon.TimeoutHandling_Return)
            if grab.GrabSucceeded():

                # image = converter.Convert(grab)
                # img = grab.GetArray()

                img = pylon.PylonImage()

                img.AttachGrabResultBuffer(grab)
                filename = "saved_pypylon_img.png"
                img.Save(pylon.ImageFileFormat_Png, filename)

                return filename

            camera.Close()


    def show_image(self):

        img_name = self.camera_basler()
        image = QPixmap(img_name)
        label = QLabel(self)
        ratio, width, height = self.compute_ratio()
        label.resize(int(ratio * width), int(ratio * height))
        label.setPixmap(image)
        label.setScaledContents(True)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.mousePressEvent = self.getPos

    def getPos(self, event):
        x = event.pos().x()
        y = event.pos().y()
        print(x, y)
        start_pt, vect = self.compute_trajectory(x, y, 40)
        print(start_pt, vect)
        print('start : ', start_pt)
        print('vect : ', vect)
        position, vecteur = self.mise_en_forme_commande_vecteur(start_pt, vect)
        print(position, vecteur)

    def compute_ratio(self):

        # Variables
        nb_images = 2
        width = 3840
        height = 2748
        fps = 300
        exposureTime = 20000
        exitCode = 0

        W = 1200
        H = 600
        ratio = min(W / width, H / height)

        return ratio, width, height

    def compute_trajectory(self, i, j, z0):
        # r0 = [100, 30, 120]
        # d = [100, 30, 120]
        r0 = lines_coef[j][i][0]
        d = lines_coef[j][i][1]
        end_t = 0
        start_t = (z0 - r0[2]) / d[2]

        end_pt = np.array(
            [[((r0[0] + d[0] * end_t) + x_offset) * x_coef], [((r0[1] + d[1] * end_t) + y_offset) * y_coef],
             [(r0[2] + d[2] * end_t) + z_offset]])
        # print("End : ", (r0[2] + d[2] * end_t))
        start_pt = np.array(
            [[((r0[0] + d[0] * start_t) + x_offset) * x_coef], [((r0[1] + d[1] * start_t) + y_offset) * y_coef],
             [(r0[2] + d[2] * start_t) + z_offset]])
        # print("Start : ", (r0[2] + d[2] * start_t))
        board_rot, jac = cv2.Rodrigues(np.array([[board_vector[3]], [board_vector[4]], [board_vector[5]]]))
        board_trans = np.array([[board_vector[0] * 100], [board_vector[1] * 100], [board_vector[2] * 100]])

        end_pt = np.dot(board_rot, end_pt) + board_trans
        start_pt = np.dot(board_rot, start_pt) + board_trans

        vect = end_pt - start_pt

        # start_pt = np.array([[(start_pt[0][0] - 0.0066) * (1 - 0.01815)], [(start_pt[1][0] - 0.03505) * (1 + 0.01845)]])

        return start_pt, vect

    def mise_en_forme_commande_vecteur(self, pos_0, vect):
        print(pos_0)
        print(vect)
        commande = f"({pos_0[0][0] * 0.01},{pos_0[1][0] * 0.01},{(pos_0[2][0]) * 0.01},3.14,0,0)"
        vecteur = f"({vect[0][0]},{vect[1][0]},{vect[2][0]})"
        print(f"COMMANDE : {commande}")
        print(f"VECTEUR : {vecteur}")

        return commande, vecteur


def main():
    application = QApplication(sys.argv)
    widget = ImageWidget()
    widget.resize(1200, 600)
    widget.setWindowTitle("Robot Ur10")
    widget.show()
    sys.exit(application.exec_())


if __name__ == '__main__':
    main()
