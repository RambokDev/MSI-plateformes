#!/usr/bin/python3
import atexit
import subprocess
import cv2
from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QMessageBox, QFileDialog
import sys
import os
from numpy import size
from robot.ur.camera.camera_connexion import camera_basler
from robot.ur.commands.start_ros_config import load_ros_config
from main_robot_connexion import robot_connexion


class Ui(QtWidgets.QMainWindow, ):
    def __init__(self):
        super(Ui, self).__init__()
        self.imageDeBase = None
        atexit.register(self.exit_handler)

        uic.loadUi(f'{os.getcwd()}/robot/ur/ihm_tests/ui/main.ui', self)
        # rospy.init_node("test_robotUR")
        self.robot_state = False
        self.myRobot = None
        # self.PIN_CAM_DEVRACAGE = 5
        # self.PIN_VENTURI_VIDE = 0
        # self.PIN_CAM_ORIENTATION = 4
        # self.ON, self.OFF = 1, 0
        # self.set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        self.take_image.clicked.connect(self.show_image)
        # self.take_image_angle.clicked.connect(self.show_image_angle)
        # self.angle_state.clicked.connect(self.action_slider_button_clicked)
        # self.go_to_box.clicked.connect(self.go_to_box_traj)
        # self.stop_robot.clicked.connect(lambda: self.robot_activation(False))
        # self.brakes.clicked.connect(lambda: self.robot_activation(True))
        self.config_button.clicked.connect(self.getfiles)
        self.quit_button.clicked.connect(self.exit_handler)
        # self.init.clicked.connect(lambda: self.myRobot.go_to_initial_position(10))
        self.start_stop_connexion.clicked.connect(self.robot_connexion)
        self.showMaximized()
        self.robot_connexion_state = False
        self.filename_config = None
        self.filename = None
        self.sensor_contact = None
        # self.slider_angle.valueChanged.connect(self.slider_state)
        self.show()

    def exit_handler(self):
        if self.filename_config is not None:
            if self.robot_state:
                success, message = robot_connexion(False)
                print(success, message)
                pid = os.getpid()
                subprocess.call(['kill', '-9', str(pid)])
        else:
            exit()

    def getfiles(self):
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.AnyFile)

        if dlg.exec_():
            filenameConfig = dlg.selectedFiles()
            self.filename_config = filenameConfig[0]
            print(self.filename_config)
            load_ros_config(filenameConfig[0])

    def robot_connexion(self):
        print(self.filename_config)
        if self.filename_config is not None:
            print(self.robot_state)
            if not self.robot_state:
                print('Start Connexion')
                success, message, robot = robot_connexion(True)
                if success:
                    self.myRobot = robot
                    self.robot_state = True
                else:
                    self.pop_up_screen(message)
            else:
                print('Stop Connexion')
                success, message = robot_connexion(False)
                if success:
                    self.robot_state = False
                else:
                    self.pop_up_screen(message)
        else:
            self.pop_up_screen('Please load the robot config file and try again')

    def pop_up_screen(self, message):
        message_box = QMessageBox()
        message_box.setIcon(QMessageBox.Critical)
        message_box.setWindowTitle("Error Message")
        message_box.setText("{}".format(message))
        message_box.setStandardButtons(QMessageBox.Ok)
        message_box.exec_()

    def show_image(self):
        """
        This function is called in order to display the image in pickup tab
        """
        print("=====Display image =====")
        img_name = camera_basler(1)
        self.filename = img_name
        self.image.setFixedSize(1385, 923)
        ratio, width, height = self.compute_ratio(1)
        image = cv2.imread(img_name)
        image = cv2.resize(image, (round(ratio * width), round(ratio * height)))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channels = image.shape
        print(height, width, channels)
        step = channels * width
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        self.image.setPixmap(QPixmap.fromImage(qImg))
        # self.image.mousePressEvent = self.getPos

    def compute_ratio(self, camera_type):
        """
        This function compute for you the ratio in order to display the right image in the qt label.
        :param camera_type: the camera type 0 or 1
        """
        if camera_type == 0:
            W = self.image_angle.width()
            H = self.image_angle.height()
        else:
            W = self.image.width()
            H = self.image.height()

        self.imageDeBase = cv2.imread(self.filename)

        width = size(self.imageDeBase, 1)
        height = size(self.imageDeBase, 0)
        ratio = min(W / width, H / height)
        print(ratio, W, H, width, height)
        return ratio, width, height


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()


if __name__ == '__main__':
    main()
