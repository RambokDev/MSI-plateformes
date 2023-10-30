import rospy
from ur_msgs.msg import IOStates


def main():
    while True:
        rospy.init_node('listener_sensor')
        msg = rospy.wait_for_message('ur_hardware_interface/io_states', IOStates)
        print(msg.digital_in_states[7].state)


if __name__ == '__main__':
    print("contact Sensor gripper")
    main()
