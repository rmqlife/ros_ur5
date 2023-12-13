#!/usr/bin/env python3
import rospy
from moveit_commander import MoveGroupCommander
from util import deg2rad, rad2deg
import numpy as np


def set_robot_joints(arm, robot_radians):
    arm.set_joint_value_target(robot_radians)
    arm.go()

def get_robot_joints(arm):      
    # Get the current joint positions
    current_joint_positions = arm.get_current_joint_values()
    return np.array(current_joint_positions)

if __name__ == '__main__':
    try:
        rospy.init_node('ur5_reset_position', anonymous=True)
        # Initialize MoveIt! commander
        shake_delta = 1
        # Create a MoveGroupCommander for the UR5 robot
        global arm
        arm = MoveGroupCommander("manipulator")

        # Define the target joint values for reset
        robot_radians =  get_robot_joints(arm)
        robot_degrees =  rad2deg(robot_radians)
        print(robot_degrees)

        if True:
            for i in range(len(robot_radians)):
                for j in [-1, 1]:
                    robot_degrees[i] += j * shake_delta
                    set_robot_joints(arm, robot_radians=deg2rad(robot_degrees))
                rospy.sleep(0.1)  # Sleep for 0.1 second between movements

    except rospy.ROSInterruptException:
        pass
