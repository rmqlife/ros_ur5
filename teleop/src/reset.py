#!/usr/bin/env python3
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
import sys
def set_ur5_position(radians):
    # Initialize the ROS node
    rospy.init_node('ur5_reset_position', anonymous=True)
    # Set the target joint values
    arm.set_joint_value_target(radians)
    # Plan and execute the motion to the reset position
    arm.go()

def deg2rad(deg):
    rad = [degree * (3.14159265359 / 180) for degree in deg]
    return rad

if __name__ == '__main__':
    try:
        # Initialize MoveIt! commander
        moveit_commander.roscpp_initialize(sys.argv)
        shake_delta = 5
        # Create a MoveGroupCommander for the UR5 robot
        global arm 
        arm = MoveGroupCommander("manipulator")
        # Define the target joint values for reset
        joint_degrees = [-200, -90, 90, 180, 270, 90]
        set_ur5_position(deg2rad(joint_degrees))
    except rospy.ROSInterruptException:
        pass
