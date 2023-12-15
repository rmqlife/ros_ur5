#!/usr/bin/env python3
import rospy, rosbag
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from omni_msgs.msg import OmniButtonEvent
from util import rad2deg, deg2rad, swap_order, reverse_sign
from shake import to_joint_trajectory, robot_pub_topic, 
from reset import RESET_POSE

# Global variables
omni_flag = False
reset_flag = False
joint_names = []
initial_omni_joints = None
current_omni_joints = None
initial_robot_joints = None
current_robot_joints = None


# Function to validate robot joints within limits
def validate_robot_joints(robot_joints, deg=45):
    min_limits = deg2rad([pose - deg for pose in RESET_POSE])
    max_limits = deg2rad([pose + deg for pose in RESET_POSE])
    validated_joints = np.clip(robot_joints, min_limits, max_limits)
    return validated_joints

# Callback functions
def robot_joint_callback(data):
    global joint_names, current_robot_joints, initial_robot_joints
    joint_names = data.name
    current_robot_joints = np.array(data.position)

    if initial_robot_joints is None:
        initial_robot_joints = current_robot_joints.copy()
        rospy.loginfo("Initial Robot States: %s", initial_robot_joints)

def omni_joint_callback(data):
    global initial_omni_joints, current_omni_joints
    current_omni_joints = np.array(data.position)

    if initial_omni_joints is None:
        initial_omni_joints = current_omni_joints.copy()
        rospy.loginfo("Initial Omni States: %s", initial_omni_joints)

def button_event_callback(data):
    global omni_flag, reset_flag, initial_omni_joints, initial_robot_joints
    if data.grey_button > 0:
        rospy.loginfo("Gray Button pressed!")
        omni_flag = not omni_flag
        initial_omni_joints = None
        initial_robot_joints = None

    if data.white_button > 0:
        rospy.loginfo("White Button pressed!")
        reset_flag = True


if __name__ == '__main__':
    try:
        rospy.init_node('ur5_shake', anonymous=True)
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, omni_joint_callback)
        rospy.Subscriber('/phantom/phantom/button', OmniButtonEvent, button_event_callback)
        rospy.Subscriber('/joint_states', JointState, robot_joint_callback)

        while not rospy.is_shutdown() and not joint_names:
            rospy.sleep(0.1)
        print(joint_names)

        pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=0)
        bag = rosbag.Bag('joint_trajectory.bag', 'w')
        while not rospy.is_shutdown():
            if omni_flag and initial_omni_joints is not None and initial_robot_joints is not None:
                delta_omni_joints = current_omni_joints - initial_omni_joints

                delta_omni_joints = swap_order(delta_omni_joints, j=0, k=2)
                delta_omni_joints = swap_order(delta_omni_joints, j=3, k=4)
                delta_omni_joints = reverse_sign(delta_omni_joints, j=0)
                delta_omni_joints = reverse_sign(delta_omni_joints, j=1)
                # delta_omni_joints = reverse_sign(delta_omni_joints, j=3)
                # delta_omni_joints = reverse_sign(delta_omni_joints, j=4)

                robot_joints = initial_robot_joints + delta_omni_joints

                print("robot at", rad2deg(initial_robot_joints))
                print('omni delta is', rad2deg(delta_omni_joints))

                # clip it
                robot_joints = validate_robot_joints(robot_joints)
                (pub, joint_names, robot_joints)

                bag.write('/scaled_pos_joint_traj_controller/command', publish_joint_trajectory(pub, joint_names, robot_joints))

            elif reset_flag:
                print("resetting ")
                reset_flag = False
                publish_joint_trajectory(pub, joint_names, deg2rad(RESET_POSE))

            rospy.sleep(0.05)

    except rospy.ROSInterruptException:
        pass