#!/usr/bin/env python3
import rospy
from moveit_commander import MoveGroupCommander
from util import deg2rad


if __name__ == '__main__':
    try:
        # Initialize MoveIt! commander
        shake_delta = 5
        # Create a MoveGroupCommander for the UR5 robot
        global arm 
        arm = MoveGroupCommander("manipulator")
        # Define the target joint values for reset
        robot_degrees = [-180, -90, 90, 180, 270, 90]

        arm.set_joint_value_target(deg2rad(robot_degrees))
        arm.go()

    except rospy.ROSInterruptException:
        pass
