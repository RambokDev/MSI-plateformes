from plateform.devices.specific.ur.hardware_actions_ur import set_pin_state_ur, impulse_pin_state_ur


def set_pin_state(robot_type, pin, state):
    if robot_type == 'ur':
        set_pin_state_ur(pin, state)
    elif robot_type == 'fanuc':
        print("Not available ")


def inpulse_pin_state(robot_type, pin, duration):
    if robot_type == 'ur':
        impulse_pin_state_ur(pin, duration)
    elif robot_type == 'fanuc':
        print("Not available ")


if __name__ == '__main__':
    ON, OFF = 1, 0
    print("hardware actions")
