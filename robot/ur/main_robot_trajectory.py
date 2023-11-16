#!/usr/bin/python3
import geometry_msgs.msg as geometry_msgs
import rospy


def cartesian_trajectory(robot, command, move, tool_position):
    if command == 'initial_position':
        duration = 5
        success, message = robot.go_to_initial_position(duration)
        return success, message
    elif type(command) == list:
        if move == 'relative':
            success, message = robot.relative_move(command[0], command[1], command[2])
            return success, message
        else:
            if tool_position == 'horizontal':
                success, message = robot.go_to_pose(geometry_msgs.Pose(
                    geometry_msgs.Vector3(command[0], command[1], command[2]),
                    robot.tool_horizontal_pose_camera
                ))
                return success, message
            elif tool_position == 'down':
                success, message = robot.go_to_pose(geometry_msgs.Pose(
                    geometry_msgs.Vector3(command[0], command[1], command[2]),
                    robot.tool_down_pose
                ))
                return success, message


def robot_trajectory(trajectory_type, robot, command, move=None, tool_position="down"):
    rospy.init_node("robotUR")

    if trajectory_type == "cartesian":
        print("========You are executing a cartesian trajectory======")
        success, message = cartesian_trajectory(robot, command, move, tool_position)
        return success, message

    elif trajectory_type == "articular":
        print("========You are executing a articular trajectory======")
    else:
        success = False
        message = "Invalid trajectory type"
        return success, message


if __name__ == '__main__':
    robot_trajectory()
