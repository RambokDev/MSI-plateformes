#!/usr/bin/python3


from plateform.robot.specific.ur import RobotUR


class Pickup(object):

    def __init__(self):
        super(Pickup, self).__init__()
        self.myRobot = None

    def run(self, robot_interface, command, vector):
        self.myRobot = robot_interface



if __name__ == '__main__':
    robot = RobotUR()
    pickup = Pickup()
    pickup.run(robot)
