#!/usr/bin/env python3
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
import sys
from util import deg2rad

def set_ur5_position(radians):
    # Initialize the ROS node
    rospy.init_node('ur5_reset_position', anonymous=True)

    # Set the target joint values
    arm.set_joint_value_target(radians)

    # Plan and execute the motion to the reset position
    arm.go()

def get_ur5_joints():    
    # Get the current joint positions
    current_joint_positions = arm.get_current_joint_values()
    return current_joint_positions

if __name__ == '__main__':
    try:
        # Initialize MoveIt! commander
        moveit_commander.roscpp_initialize(sys.argv)
        shake_delta = 5
        # Create a MoveGroupCommander for the UR5 robot
        global arm
        arm = MoveGroupCommander("manipulator")


        # Define the target joint values for reset
        # joint_degrees =  get_ur5_joints()
        joint_degrees = [-200, -90, 90, 180, 270, 90]

        for i in range(len(joint_degrees)):
            for j in [-1, 1]:
                joint_degrees[i] += j * shake_delta
                set_ur5_position(deg2rad(joint_degrees))
            rospy.sleep(0.1)  # Sleep for 0.1 second between movements

    except rospy.ROSInterruptException:
        pass
