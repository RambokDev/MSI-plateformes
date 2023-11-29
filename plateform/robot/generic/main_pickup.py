#!/usr/bin/python3
from plateform.devices.generic.hardware_actions import set_pin_state
from plateform.robot.specific.ur.external.sensor_loop import sensor_loop
from plateform.robot.specific.ur.main_robot_trajectory import robot_trajectory


class Pickup(object):

    def __init__(self):
        super(Pickup, self).__init__()
        self.myRobot = None

    def run(self, robot_interface, command, vector):
        self.myRobot = robot_interface
        success, message = self.go_pick_and_place(command, vector)
        return success, message

    def go_pick_and_place(self, robot_command, vector):
        """
        This function is used for the pick and place action
        :param robot_command: the robot command
        :param vector: the vector to move the robot to pick and place pump
        """
        self.sensor_contact = sensor_loop()
        success, message = robot_trajectory("cartesian", self.myRobot, "initial_position", None, "down")
        if success:
            robot_trajectory("cartesian", self.myRobot, robot_command, None, "down")
            if self.sensor_contact != 1:
                set_pin_state("ur", 0, True)
                robot_trajectory("cartesian", self.myRobot, vector, "relative", "down")
                success, message = self.go_to_camera()
                return success, message
        else:
            return success, message

    def go_to_camera(self):
        """
        This function allowed you to go to the position of the angle camera
        """
        camera_command = [-0.883, 0.775, 0.594]
        success, message = robot_trajectory("cartesian", self.myRobot, camera_command, None, "horizontal")
        return success, message


if __name__ == '__main__':
    print("main pickup class")
#     robot = RobotUR()
#     pickup = Pickup()
#     pickup.run(robot)
