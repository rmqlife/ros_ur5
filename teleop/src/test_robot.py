#!/usr/bin/env python3
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
import copy
import sys

def move_ur5_joint_back_and_forth():
    # Initialize the ROS node
    rospy.init_node('ur5_joint_test', anonymous=True)
    
    # Initialize MoveIt! commander
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Create a MoveGroupCommander for the UR5 robot
    arm = MoveGroupCommander("manipulator")

    # Set a small joint increment for movement
    joint_increment = 0.05  # Adjust this value as needed
    
    # Get the current joint positions
    current_joint_positions = arm.get_current_joint_values()

    try:
        while not rospy.is_shutdown():
            # Iterate through each joint and move it back and forth
            for i in range(len(current_joint_positions)):
                for sign in [-1, 1]:
                    # Create a copy of the current joint positions
                    new_joint_positions = copy.deepcopy(current_joint_positions)
                    new_joint_positions[i] += sign * joint_increment
                    
                    # Move the arm to the new joint positions
                    arm.set_joint_value_target(new_joint_positions)
                    arm.go()
                    
                    # Wait for a moment to reach the new position
                    rospy.sleep(0.2)  # Adjust sleep duration as needed

                rospy.sleep(1.0)

    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt. Stopping the UR5 movement.")

if __name__ == '__main__':
    try:
        move_ur5_joint_back_and_forth()
    except rospy.ROSInterruptException:
        pass
