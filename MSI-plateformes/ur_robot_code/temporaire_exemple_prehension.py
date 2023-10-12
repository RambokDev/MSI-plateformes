# Import necessary libraries
import rospy
import moveit_commander
from std_msgs.msg import String
import sys


# Declare global variables
stop_movement = False


# Define callback function for subscriber
def event_callback(data):
    global stop_movement
    if data.data == "stop":
        stop_movement = True


def prehension():
    
    # Initialize node
    rospy.init_node('moveit_example')

    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create group commander
    group = moveit_commander.MoveGroupCommander("arm")

    # Create subscriber
    event_sub = rospy.Subscriber("event_topic", String, event_callback)

    # Create publisher
    move_pub = rospy.Publisher("move_topic", String, queue_size=10)

    # Set target pose or joint values
    group.set_pose_target(...)
    # or
    group.set_joint_value_target(...)

    # Plan and execute movement
    group.go(wait=True)

    # Wait for movement to finish or for stop signal
    while not stop_movement and not group.get_state() == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
        rospy.sleep(0.1)

    # Stop movement
    group.stop()

    # Send stop signal via publisher
    move_pub.publish("stopped")

    # Clean up moveit_commander
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


# # Since we passed in wait=False above we need to wait here
#     move_group.get_move_action().wait_for_result()
#     result = move_group.get_move_action().get_result()

#     if result:
#         # Checking the MoveItErrorCode
#         if result.error_code.val == MoveItErrorCodes.SUCCESS:
#             rospy.loginfo("Disco!")