#!/usr/bin/env python3
import rospy
from util import rad2deg, deg2rad  # Import your rad2deg and deg2rad functions
from shake import myRobot

# reset pose
RESET_POSE = [-180, -90, 90, 270, 270, 180]

if __name__ == '__main__':
    try:
        rospy.init_node('reset', anonymous=True)
        my_robot = myRobot()  # Initialize the robot object

        current_joint_degrees = rad2deg(my_robot.joint_positions)
        print("current robot degrees", current_joint_degrees)

        my_robot.move_joints(deg2rad(current_joint_degrees))  # Corrected parentheses here
        rospy.sleep(0.5)
        my_robot.move_joints(deg2rad(RESET_POSE))  # Corrected parentheses here
        rospy.sleep(0.5)
    except rospy.ROSInterruptException:
        pass
