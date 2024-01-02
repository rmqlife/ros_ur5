#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from util import rad2deg, deg2rad, swap_order, reverse_sign

from reset import RESET_POSE
from echo_omni import myOmni
from shake import myRobot

# Function to validate robot joints within limits
def validate_robot_joints(robot_joints, deg=90):
    min_limits = deg2rad([pose - deg for pose in RESET_POSE])
    max_limits = deg2rad([pose + deg for pose in RESET_POSE])
    validated_joints = np.clip(robot_joints, min_limits, max_limits)
    return validated_joints

# Function to validate robot joints within limits
def validate_robot_joints(robot_joints, deg=90):
    min_limits = deg2rad([pose - deg for pose in RESET_POSE])
    max_limits = deg2rad([pose + deg for pose in RESET_POSE])
    validated_joints = np.clip(robot_joints, min_limits, max_limits)
    return validated_joints

if __name__ == '__main__':
    try:
        rospy.init_node('apply_delta_test', anonymous=True)
        my_omni = myOmni()  # Initialize the omni object
        init_flag = False
        init_flag = False

        my_robot = myRobot()  # Initialize the robot object
        initial_robot_joints = my_robot.joint_positions

        print('press gray button to start function')
        print('press gray button to start function')
        while not rospy.is_shutdown():
            if my_omni.gray_button_flag:
                if not init_flag:
                    print("init omni position")
                    initial_omni_joints = my_omni.joint_positions
                    initial_robot_joints = my_robot.joint_positions
                    init_flag = True
            if my_omni.gray_button_flag:
                if not init_flag:
                    print("init omni position")
                    initial_omni_joints = my_omni.joint_positions
                    initial_robot_joints = my_robot.joint_positions
                    init_flag = True

                print("robot at", rad2deg(my_robot.joint_positions))
                print("omni at", rad2deg(my_omni.joint_positions))
                delta_omni_joints = my_omni.joint_positions - initial_omni_joints
                print('omni delta is', rad2deg(delta_omni_joints))
                print("robot at", rad2deg(my_robot.joint_positions))
                print("omni at", rad2deg(my_omni.joint_positions))
                delta_omni_joints = my_omni.joint_positions - initial_omni_joints
                print('omni delta is', rad2deg(delta_omni_joints))

                # Switch the unwanted order:
                # delta_omni_joints = swap_order(delta_omni_joints, j=0, k=2)
                delta_omni_joints = swap_order(delta_omni_joints, j=3, k=4)
                delta_omni_joints = reverse_sign(delta_omni_joints, j=1)
                delta_omni_joints = reverse_sign(delta_omni_joints, j=2)

                robot_joints = initial_robot_joints + delta_omni_joints
                robot_joints = validate_robot_joints(robot_joints)
                
                my_robot.move_joints(robot_joints)

            else: # clean the initial postion of omni phantom
                init_flag = False
                if my_omni.white_button_flag:
                    print('resetting robot')
                    my_omni.white_button_flag = not my_omni.white_button_flag
                    my_robot.move_joints(deg2rad(RESET_POSE))
            rospy.sleep(0.03)
                robot_joints = initial_robot_joints + delta_omni_joints
                robot_joints = validate_robot_joints(robot_joints)
                
                my_robot.move_joints(robot_joints)

            else: # clean the initial postion of omni phantom
                init_flag = False
                if my_omni.white_button_flag:
                    print('resetting robot')
                    my_omni.white_button_flag = not my_omni.white_button_flag
                    my_robot.move_joints(deg2rad(RESET_POSE))
            rospy.sleep(0.03)

    except rospy.ROSInterruptException:
        pass






