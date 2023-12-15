#!/usr/bin/env python3
import rospy
from util import rad2deg, deg2rad  # Import your rad2deg and deg2rad functions
from shake_new import myRobot

# reset pose
RESET_POSE = [90, -90, -180, 270, 270, 90]

if __name__ == '__main__':
    try:
        my_robot = myRobot()  # Initialize the robot object

        if my_robot.current_joint_positions:
            current_joint_degrees = rad2deg(my_robot.current_joint_positions)
            print("current robot degrees", current_joint_degrees)

        my_robot.move_joints(deg2rad(current_joint_degrees))  # Corrected parentheses here
        rospy.sleep(0.5)
        my_robot.move_joints(deg2rad(RESET_POSE))  # Corrected parentheses here
        rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        pass
