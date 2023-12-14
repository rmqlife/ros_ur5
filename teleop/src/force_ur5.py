#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
from util import rad2deg, deg2rad, swap_order, reverse_sign
from shake import publish_joint_trajectory

# Global variable to store joint names and positions
joint_names = []
ur5_joint_publisher = None

# Define a global variable to store the initial joint positions
initial_omni_joints = None
current_omni_joints = []
initial_robot_joints = None
current_robot_joints = []

def robot_joint_callback(data):
    # Store the received joint states in the global variables
    global joint_names, current_robot_joints, initial_robot_joints
    joint_names = data.name
    current_robot_joints = np.array(data.position)

    if initial_robot_joints is None:
        initial_robot_joints = current_robot_joints.copy()
        rospy.loginfo("Initial Robot States: %s", initial_robot_joints)
        return

    pass

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

        # Create a subscriber to the '/joint_states' topic
        robot_joint_subscriber = rospy.Subscriber('/joint_states', JointState, robot_joint_callback)

        # Wait for the subscriber to receive joint names
        while not rospy.is_shutdown() and not joint_names:
            rospy.sleep(0.1)
        print(joint_names)

        # Create a publisher for the '/scaled_pos_joint_traj_controller/command' topic
        pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

        while not rospy.is_shutdown():
            print("runnning")

            print("robot at", rad2deg(current_robot_joints))
            delta_robot_joints = current_robot_joints - initial_robot_joints
            print("robot delta is", rad2deg(delta_robot_joints))

            print("omni at", rad2deg(current_omni_joints))
            delta_omni_joints = current_omni_joints - initial_omni_joints
            print('omni delta is', rad2deg(delta_omni_joints))

            # switch the unwanted order:
            delta_omni_joints = swap_order(delta_omni_joints, j=0, k=2)
            delta_omni_joints = swap_order(delta_omni_joints, j=3, k=4)
            
            delta_omni_joints = reverse_sign(delta_omni_joints, j=0)
            delta_omni_joints = reverse_sign(delta_omni_joints, j=1)
            # delta_omni_joints = reverse_sign(delta_omni_joints, j=2)
            
            robot_joints = initial_robot_joints + delta_omni_joints
            
            publish_joint_trajectory(pub, joint_names, robot_joints)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass

