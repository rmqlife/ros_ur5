#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from util import rad2deg, deg2rad, swap_order, reverse_sign

from reset import RESET_POSE
from echo_omni import myOmni
from shake import myRobot

if __name__ == '__main__':
    try:
        rospy.init_node('apply_delta_test', anonymous=True)
        my_omni = myOmni()  # Initialize the omni object
        initial_omni_joints = my_omni.joint_positions

        my_robot = myRobot()  # Initialize the robot object
        initial_robot_joints = my_robot.joint_positions

        while not rospy.is_shutdown():
            print("running")

            print("robot at", rad2deg(my_robot.joint_positions))

            print("omni at", rad2deg(my_omni.joint_positions))
            delta_omni_joints = my_omni.joint_positions - initial_omni_joints
            print('omni delta is', rad2deg(delta_omni_joints))

            # Switch the unwanted order:
            delta_omni_joints = swap_order(delta_omni_joints, j=0, k=2)
            delta_omni_joints = swap_order(delta_omni_joints, j=3, k=4)
            delta_omni_joints = reverse_sign(delta_omni_joints, j=0)
            delta_omni_joints = reverse_sign(delta_omni_joints, j=1)

            robot_joints = initial_robot_joints + delta_omni_joints

            my_robot.move_joints(robot_joints)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
