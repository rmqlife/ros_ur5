#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
from omni_msgs.msg import OmniButtonEvent

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
RESET_POSE = [90, -90, -180, 180, 270, 90]

omni_flag=False
reset_flag=False
def button_event_callback(data):
    global omni_flag, reset_flag, initial_omni_joints, initial_robot_joints
    # Process the button event
    if data.grey_button>0:
        rospy.loginfo("Gray Button pressed!")
        omni_flag=not omni_flag; #True
        # reset initial omni joints and robot joints
        initial_omni_joints = None
        initial_robot_joints = None

    if data.white_button>0:
        rospy.loginfo("White Button pressed!")
        # reset initial robot joints
        reset_flag = True


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
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, omni_joint_callback)
        rospy.Subscriber('/phantom/phantom/button', OmniButtonEvent, button_event_callback)

        # Create a subscriber to the '/joint_states' topic
        rospy.Subscriber('/joint_states', JointState, robot_joint_callback)

        # Wait for the subscriber to receive joint names
        while not rospy.is_shutdown() and not joint_names:
            rospy.sleep(0.1)
        print(joint_names)

        # Create a publisher for the '/scaled_pos_joint_traj_controller/command' topic
        pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

        while not rospy.is_shutdown():
            if omni_flag and initial_omni_joints is not None and initial_robot_joints is not None:
                print("runnning")

                delta_robot_joints = current_robot_joints - initial_robot_joints

                if False:
                    print("robot at", rad2deg(current_robot_joints))
                    print("robot delta is", rad2deg(delta_robot_joints))

                delta_omni_joints = current_omni_joints - initial_omni_joints
                if True:
                    print("omni at", rad2deg(current_omni_joints))
                    print('omni delta is', rad2deg(delta_omni_joints))

                # switch the unwanted order:
                delta_omni_joints = swap_order(delta_omni_joints, j=0, k=2)
                delta_omni_joints = swap_order(delta_omni_joints, j=3, k=4)
                
                delta_omni_joints = reverse_sign(delta_omni_joints, j=0)
                delta_omni_joints = reverse_sign(delta_omni_joints, j=1)
                
                robot_joints = initial_robot_joints + delta_omni_joints
                publish_joint_trajectory(pub, joint_names, robot_joints)

            elif reset_flag:
                print("resetting ")
                reset_flag = False
                publish_joint_trajectory(pub, joint_names, deg2rad(RESET_POSE))
            
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass

