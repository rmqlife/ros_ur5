#!/usr/bin/env python3
import rospy
from util import rad2deg, deg2rad  # Import your rad2deg and deg2rad functions
from myRobot import MyRobot

# reset pose
RESET_POSE_DEG = [-180, -90, 90, 270, 270, 90]
RESET_POSE_RAD = deg2rad(RESET_POSE_DEG)

if __name__ == '__main__':
    try:
        rospy.init_node('reset', anonymous=True)
        my_robot = MyRobot()  # Initialize the robot object

        joints = my_robot.get_joints()
        print("current robot degrees", joints)
        print('reset pose', RESET_POSE_RAD)

        my_robot.move_joints(RESET_POSE_RAD, duration=1)  # Corrected parentheses here
    except rospy.ROSInterruptException:
        pass
