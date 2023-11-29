#!/usr/bin/python3

class Platform(object):

    def __init__(self):
        super(Platform, self).__init__()
        self.Robot_interface = None
        self.Robot_params = None

    def storeParams(self, robot_params):
        print("I am storing my platforms data")
        self.Robot_params = robot_params

    def storeInterface(self, robot_interface):
        print("I am storing my platforms data")
        self.Robot_interface = robot_interface


if __name__ == '__main__':
    print("Storing DATA ")
