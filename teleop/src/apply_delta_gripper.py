#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from util import rad2deg, deg2rad, swap_order, reverse_sign

from myOmni import MyOmni
from myRobot import MyRobot

from myIO import MyIO

if __name__ == '__main__':
    try:
        rospy.init_node('apply_delta', anonymous=True)
        my_omni = MyOmni()  # Initialize the omni object
        my_robot = MyRobot()  # Initialize the robot object
        my_io = MyIO(1, 16)
        teleop_sign = False
        hold_to_control=False
    
        print('press gray button to start function')
        while not rospy.is_shutdown():

            teleop_sign_prev = teleop_sign
            if hold_to_control:
                teleop_sign = my_omni.gray_button
            else:
                teleop_sign = my_omni.gray_button_flag
            
            # Check if the gray button state has changed and recording is not active
            if teleop_sign and not teleop_sign_prev:
                my_robot.start_bag_recording()
                initial_omni_joints = my_omni.joint_positions
                initial_robot_joints = my_robot.joint_positions

            # Check if the gray button state has changed and recording is active
            elif not teleop_sign and teleop_sign_prev:
                my_robot.stop_bag_recording()
                
            # Print joint states while recording is active
            if teleop_sign:
                omni_joints = my_omni.get_joints()
                print("robot at", rad2deg(my_robot.joint_positions))
                print("omni at", rad2deg(my_omni.joint_positions))
                delta_omni_joints = my_omni.joint_positions - initial_omni_joints
                print('omni delta is', rad2deg(delta_omni_joints))
                print("robot at", rad2deg(my_robot.joint_positions))

                # Switch the unwanted order:
                delta_omni_joints = swap_order(delta_omni_joints, j=3, k=4)
                delta_omni_joints = reverse_sign(delta_omni_joints, j=1)
                delta_omni_joints = reverse_sign(delta_omni_joints, j=2)

                robot_joints = initial_robot_joints + delta_omni_joints
                # compute a distance 
                dist = np.linalg.norm(delta_omni_joints)
                print('distance', dist)
                my_robot.move_joints(robot_joints, duration=0.1)
            
            # control the gripper
            if my_omni.white_button_flag:
                my_omni.white_button_flag = not my_omni.white_button_flag
                print('clicked white button')
                my_io.toggle_state()

            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        pass
