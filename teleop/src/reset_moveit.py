#!/usr/bin/env python3
import rospy
from moveit_commander import MoveGroupCommander
from util import deg2rad
from shake_moveit import set_robot_joints

if __name__ == '__main__':
    try:
        # Create a MoveGroupCommander for the UR5 robot
        arm = MoveGroupCommander("manipulator")
        # Define the target joint values for reset
        robot_degrees = [-180, -90, 90, 180, 270, 90]
        set_robot_joints(arm, deg2rad(robot_degrees))

    except rospy.ROSInterruptException:
        pass
