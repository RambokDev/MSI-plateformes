#!/usr/bin/python3
import time
import roslaunch
import json


def start_ros_config(data_config):
    """
    This function is called after loading the config file
    This start the ROS config
    """
    print(data_config)
    robot_ip = data_config['robot_ip']
    launch_file_path = data_config['launch_file_path']
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    cli_args = [launch_file_path, 'robot_ip:={}'.format(robot_ip)]
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start()
    time.sleep(2)


def load_ros_config(config_file):
    """
    This function is called in order to load the ros config file
    """
    config_file = open(config_file, 'r')

    try:
        data_config = json.load(config_file)
        start_ros_config(data_config)
        config_file.close()
    except ValueError as e:
        return False, "Error loading config file: {}".format(e)


if __name__ == '__main__':
    config_file_path = "/robot/config/config_ur.json"
    load_ros_config(config_file=config_file_path)
