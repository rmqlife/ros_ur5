#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from shake_new import myRobot
from util import rad2deg, deg2rad, swap_order, reverse_sign
from shake import publish_joint_trajectory
from phantom_omni.msg import OmniButtonEvent  # Make sure to import OmniButtonEvent if it's from the phantom_omni package

# reset pose
RESET_POSE = [90, -90, -180, 270, 270, 90]

# Global variables for joint names and positions
joint_names = []
omni_flag = False
reset_flag = False
initial_omni_joints = None
current_omni_joints = []
initial_robot_joints = None
current_robot_joints = []

class myOmni:
    def __init__(self):
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.omni_joint_callback)
        rospy.Subscriber('/phantom/phantom/button', OmniButtonEvent, self.button_event_callback)


        self.            initial_omni_joints = current_omni_joints.copy()
            rospy.loginfo("Initial Omni States: %s", initial_omni_joints)



    def omni_joint_callback(self, data):
        global initial_omni_joints, current_omni_joints
        current_omni_joints = np.array(data.position)



    def button_event_callback(self, data):
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
        my_omni = myOmni()  # Initialize the omni object

        my_robot = myRobot()  # Initialize the robot object

        while not rospy.is_shutdown():
            print("running")

            print("robot at", rad2deg(my_robot.current_robot_joints))
            delta_robot_joints = my_robot.current_robot_joints - my_robot.initial_robot_joints
            print("robot delta is", rad2deg(delta_robot_joints))

            print("omni at", rad2deg(current_omni_joints))
            delta_omni_joints = current_omni_joints - initial_omni_joints
            print('omni delta is', rad2deg(delta_omni_joints))

            # Switch the unwanted order:
            delta_omni_joints = swap_order(delta_omni_joints, j=0, k=2)
            delta_omni_joints = swap_order(delta_omni_joints, j=3, k=4)

            delta_omni_joints = reverse_sign(delta_omni_joints, j=0)
            delta_omni_joints = reverse_sign(delta_omni_joints, j=1)
            # delta_omni_joints = reverse_sign(delta_omni_joints, j=2)

            robot_joints = my_robot.initial_robot_joints + delta_omni_joints

            my_robot.publish_joint_trajectory(my_robot.pub, my_robot.joint_names, robot_joints)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
