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
from ihm_tests.camera.camera_connexion import camera_basler
from plateform.devices.generic.hardware_actions import set_pin_state
from plateform.robot.generic.main_pickup import Pickup
from plateform.robot.generic.start_ros_config import load_ros_config
from plateform.robot.specific.ur.commands.trajectory.compute_trajectory import compute_trajectory, formatting_commands
from plateform.robot.specific.ur.external.sensor_loop import sensor_loop
from plateform.robot.generic.main_robot_connexion import robot_connexion
from plateform.robot.specific.ur.main_robot_trajectory import robot_trajectory, robot_get_info, robot_create_quaternions
from plateform.robot.specific.ur.plateform_class import Platform


class Ui(QtWidgets.QMainWindow, ):
    def __init__(self):
        super(Ui, self).__init__()
        self.imageDeBase = None
        atexit.register(self.exit_handler)
        uic.loadUi(f'{os.getcwd()}/ihm_tests/ui/main.ui', self)
        self.robot_state = False
        self.myRobot = None
        self.take_image.clicked.connect(self.show_image)
        self.take_image_angle.clicked.connect(self.show_image_angle)
        self.angle_state.clicked.connect(self.action_slider_button_clicked)
        self.go_to_box.clicked.connect(self.go_to_box_traj)
        self.config_button.clicked.connect(self.getfiles)
        self.quit_button.clicked.connect(self.exit_handler)
        self.init.clicked.connect(self.go_to_init)
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
                robot_connexion(False)
                sys.exit()
        else:
            sys.exit()

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
                    self.state_connexion_robot.setText("Connected")
                    self.state_connexion_robot.setStyleSheet("background-color: green;padding :15px;color:white")
                    return success
                else:
                    self.pop_up_screen(message)
                    return success
            else:
                print('Stop Connexion')
                success, message = robot_connexion(False)
                if success:
                    self.robot_state = False
                    self.state_connexion_robot.setText("Disconnected")
                    self.state_connexion_robot.setStyleSheet("background-color: red;padding :15px;color:white")
                    subprocess.call(['killall', '-9', 'rosmaster'])
                    return success
                else:
                    self.pop_up_screen(message)
                    return success
        else:
            success = False
            self.pop_up_screen('Please load the robot config file and try again')
            return success

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
        self.image.mousePressEvent = self.getPos

    def show_image_angle(self):
        """
        This function is called in order to display the image in angle tab
        """
        print("=====Display image Angle =====")
        img_name_angle = camera_basler(0)
        self.image_angle.setFixedSize(1385, 923)
        ratio, width, height = self.compute_ratio(0)
        image = cv2.imread(img_name_angle)
        image = cv2.resize(image, (round(ratio * width), round(ratio * height)))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channels = image.shape
        print(height, width, channels)
        step = channels * width
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        self.image_angle.setPixmap(QPixmap.fromImage(qImg))

    def getPos(self, event):
        """
        This function compute the mouse position and convert the image x,y in camera coordinates
        :param event: the mouse event
        """
        x = event.pos().x()
        y = event.pos().y()
        ratio, width, height = self.compute_ratio(1)
        print(ratio)
        print("coordinates in IHM picture: ", x, y)
        realX = round(x / ratio)
        realY = round(y / ratio)
        print("coordinates in Real : ", realX, realY)

        # robot
        start_pt, vect = compute_trajectory(realX, realY, 40)
        print(start_pt, vect)
        print('start : ', start_pt)
        print('vect : ', vect)
        robot_command, vector = formatting_commands(start_pt, vect)
        print('robot_command : ', robot_command)
        print('vector : ', vector)
        self.go_pick_and_place(robot_command, vector)

    def go_pick_and_place(self, robot_command, vector):
        """
        This function is used for the pick and place action
        :param robot_command: the robot command
        :param vector: the vector to move the robot to pick and place pump
        """

        print("interface", Platform().Robot_interface)
        print("params", Platform().Robot_params)

        success, message = Pickup().run(Platform().Robot_interface, robot_command, vector)
        print(success, message)

    def go_to_init(self):
        success, message = robot_trajectory("cartesian", self.myRobot, "initial_position", None, "down")

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

    def action_slider_button_clicked(self):
        print("The slider angle is : ", self.slider_angle.value())
        wrist_angle = self.slider_angle.value()

        pose_camera = [-61.86, -48.79, 65.73, -198.09, -32.20, wrist_angle]
        success, message = robot_trajectory("articular", self.myRobot, pose_camera)

    def go_to_box_traj(self):
        """
        This function is called in order to go to the box trajectory,
        normally the venturi is already start
        """
        current_pose = robot_get_info("current_pose", self.myRobot)
        camera_command = [-0.883, 0.775, 0.300]
        current_quaternions = robot_create_quaternions(current_pose)
        success, message = robot_trajectory("cartesian", self.myRobot, camera_command, None, current_quaternions)
        set_pin_state("ur", 0, False)
        prepare_command_wrist = [-61.88, -35.19, 73.40, -219.29, -32.29, 190]
        if success:
            success, message = robot_trajectory("articular", self.myRobot, prepare_command_wrist)
            if success:
                success, message = robot_trajectory("cartesian", self.myRobot, "initial_position", None, "down")


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()


if __name__ == '__main__':
    main()
