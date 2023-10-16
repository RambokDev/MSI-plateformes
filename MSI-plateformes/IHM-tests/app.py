from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from PyQt5.QtGui import QPixmap
import sys
import numpy as np
import cv2 as cv
import os
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

    def show_image(self):
        image = QPixmap("test_box.jpg")
        print(image)
        label = QLabel(self)
        label.setPixmap(image)
        label.resize(1200, 600)
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
        board_rot, jac = cv.Rodrigues(np.array([[board_vector[3]], [board_vector[4]], [board_vector[5]]]))
        board_trans = np.array([[board_vector[0] * 100], [board_vector[1] * 100], [board_vector[2] * 100]])

        end_pt = np.dot(board_rot, end_pt) + board_trans
        start_pt = np.dot(board_rot, start_pt) + board_trans

        vect = end_pt - start_pt

        start_pt = np.array([[(start_pt[0][0] - 0.0066) * (1 - 0.01815)], [(start_pt[1][0] - 0.03505) * (1 + 0.01845)]])

        return start_pt, vect


def main():
    application = QApplication(sys.argv)
    widget = ImageWidget()
    widget.resize(1200, 600)
    widget.setWindowTitle("Test")
    widget.show()
    sys.exit(application.exec_())


if __name__ == '__main__':
    main()
