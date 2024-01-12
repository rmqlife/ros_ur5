#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
import sys
from util import rad2deg, deg2rad, swap_order, reverse_sign
from shake_moveit import get_robot_joints, set_robot_joints
from moveit_commander import MoveGroupCommander

# Define a global variable to store the initial joint positions
initial_omni_joints = None
current_omni_joints = []


def omni_joint_callback(data):
    global initial_omni_joints, current_omni_joints
    current_omni_joints = np.array(data.position)

    if initial_omni_joints is None:
        initial_omni_joints = current_omni_joints.copy()
        rospy.loginfo("Initial Omni States: %s", initial_omni_joints)
        return
    pass

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('ur5_shake', anonymous=True)
        omni_joint_subscriber = rospy.Subscriber('/phantom/phantom/joint_states', JointState, omni_joint_callback)

        arm = MoveGroupCommander("manipulator")
        initial_robot_joints = get_robot_joints(arm)


        while not rospy.is_shutdown():
            if initial_omni_joints is not None and initial_robot_joints is not None:
                print("runnning")

                current_robot_joints = get_robot_joints(arm)
                print("robot at", rad2deg(current_robot_joints))
                delta_robot_joints = current_robot_joints - initial_robot_joints
                print("robot delta is", rad2deg(delta_robot_joints))

                print("omni at", rad2deg(current_omni_joints))
                delta_omni_joints = current_omni_joints - initial_omni_joints
                print('omni delta is', rad2deg(delta_omni_joints))

                # switch the unwanted order:
                delta_omni_joints = swap_order(delta_omni_joints, j=3, k=4)
                
                # delta_omni_joints = reverse_sign(delta_omni_joints, j=0)
                # delta_omni_joints = reverse_sign(delta_omni_joints, j=1)
                # delta_omni_joints = reverse_sign(delta_omni_joints, j=2)
                
                robot_joints = initial_robot_joints + delta_omni_joints
                
                set_robot_joints(arm, robot_joints)

                rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        pass

