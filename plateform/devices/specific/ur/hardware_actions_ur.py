import rospy
from ur_msgs.srv import SetIO



def set_pin_state_ur(pin: int, state: bool):
    set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    set_io_interface(1, pin, state)


def impulse_pin_state_ur(pin: int, time: float):
    set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    set_io_interface(1, pin, 1)
    rospy.sleep(time)
    set_io_interface(1, pin, 0)





if __name__ == '__main__':
    ON, OFF = 1, 0
    print("hardware actions ur ")
