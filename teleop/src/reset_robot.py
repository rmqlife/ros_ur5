#!/usr/bin/env python3
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
import sys
def reset_ur5_position():
    # Initialize the ROS node
    rospy.init_node('ur5_reset_position', anonymous=True)

    # Initialize MoveIt! commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Create a MoveGroupCommander for the UR5 robot
    arm = MoveGroupCommander("manipulator")

    # Define the target joint values for reset
    joint_degrees = [-180, -90, 90, 180, 270, 90]
    joint_radians = [degree * (3.14159265359 / 180) for degree in joint_degrees]

    # Set the target joint values
    arm.set_joint_value_target(joint_radians)

    # Plan and execute the motion to the reset position
    arm.go()

if __name__ == '__main__':
    try:
        reset_ur5_position()
    except rospy.ROSInterruptException:
        pass
