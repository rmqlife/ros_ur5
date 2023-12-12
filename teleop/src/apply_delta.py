#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal
from actionlib_msgs.msg import GoalStatus
import sys
from util import rad2deg, deg2rad




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

def swap_order(i, j, k):
    i[j], i[k] = i[k], i[j]
    return i

def reverse_sign(i, j):
    i[j] = -i[j]
    return i

def omni_joint_callback(data):
    global initial_omni_joints, current_omni_joints
    current_omni_joints = np.array(data.position)


    if initial_omni_joints is None:
        initial_omni_joints = current_omni_joints.copy()
        rospy.loginfo("Initial Omni States: %s", initial_omni_joints)
        return

    pass

def publish_joint_trajectory(pub, joint_positions):
    if not joint_names:
        rospy.logwarn("No joint names available yet. Waiting for joint_states message.")
        return

    if len(joint_positions) != len(joint_names):
        rospy.logerr("Joint positions do not match the number of joint names.")
        return

    # Create a JointTrajectory message
    joint_traj = JointTrajectory()

    # Set the joint names from the received message
    joint_traj.joint_names = joint_names

    # Create a JointTrajectoryPoint for the desired joint positions
    point = JointTrajectoryPoint()
    point.positions = joint_positions

    # Set the time from start for this point (you can adjust this)
    point.time_from_start = rospy.Duration(0.1)  # 0.1 seconds between each point

    # Append the JointTrajectoryPoint to the trajectory
    joint_traj.points.append(point)

    # Publish the JointTrajectory message
    pub.publish(joint_traj)


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


            # apply the delta to robot:
            # desired robot_joint
            # switch the unwanted order:
            delta_omni_joints = swap_order(delta_omni_joints, j=0, k=2)
            delta_omni_joints = swap_order(delta_omni_joints, j=3, k=4)
            
            delta_omni_joints = reverse_sign(delta_omni_joints, j=0)
            # delta_omni_joints = reverse_sign(delta_omni_joints, j=2)
            # delta_omni_joints = reverse_sign(delta_omni_joints, j=3)
            delta_omni_joints = reverse_sign(delta_omni_joints, j=1)

            
            robot_joints = initial_robot_joints + delta_omni_joints
            
            
            publish_joint_trajectory(pub, robot_joints)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass

