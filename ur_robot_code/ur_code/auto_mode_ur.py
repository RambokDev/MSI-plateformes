import time
import functions
from math import pi, tau, dist, fabs, cos

PIN_VENTURI_VIDE = 0
PIN_CAM_ORIENTATION = 4
PIN_CAM_DEVRACAGE = 5
ON, OFF = 1, 0


def auto_mode_ur(robot_interface, set_io_interface, init_position,
                 pickup_position, camera_position,camera_position_in,box_position):
    print("==============Auto mode              =================")
    # print("==============Go to init mode        =================")
    # robot_interface.go_to_joint_state(functions.convert_deg_to_rad(init_position))

    # print("==============Go to take the gripper =================")
    # robot_interface.go_to_joint_state(functions.convert_deg_to_rad(prehensor_position))
    # robot_interface.go_to_joint_state(functions.convert_deg_to_rad(prehensor_position_out))

    print("==============Go to waiting position =================")
    robot_interface.go_to_joint_state(functions.convert_deg_to_rad(init_position))
    set_io_interface(1, PIN_CAM_DEVRACAGE, ON)
    time.sleep(0.5)
    set_io_interface(1, PIN_CAM_DEVRACAGE, OFF)
    print("==============Go to pickup object    =================")
    robot_interface.go_to_joint_state(functions.convert_deg_to_rad(pickup_position))

    print("==============Start Venturi          =================")
    set_io_interface(1, PIN_VENTURI_VIDE, ON)
    time.sleep(5)

    print("==============GO to waiting position =================")
    robot_interface.go_to_joint_state(functions.convert_deg_to_rad(init_position))

    print("==============Go to camera position  =================")
    robot_interface.go_to_joint_state(functions.convert_deg_to_rad(camera_position))
    robot_interface.go_to_joint_state(functions.convert_deg_to_rad(camera_position_in))

    print("==============Rotation for camera    =================")
    set_io_interface(1, PIN_CAM_ORIENTATION, ON)
    time.sleep(0.5)
    set_io_interface(1, PIN_CAM_ORIENTATION, OFF)
    angle_correction_deg = -90
    angle_correction_rad = angle_correction_deg * pi / 180
    joint_goal = robot_interface.move_group.get_current_joint_values()
    joint_goal[5] += angle_correction_rad
    robot_interface.move_group.go(joint_goal, wait=True)
    robot_interface.move_group.stop()
    print("==============go to box position     =================")
    robot_interface.go_to_joint_state(functions.convert_deg_to_rad(box_position))
    print("==============Stop  Venturi          =================")
    set_io_interface(1, PIN_VENTURI_VIDE, OFF)
    print("==============Go to init mode        =================")
    robot_interface.go_to_joint_state(functions.convert_deg_to_rad(init_position))
    print("==============End                    =================")


if __name__ == "__main__":
    auto_mode_ur()
