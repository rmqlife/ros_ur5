#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniButtonEvent

class myOmni:
    def __init__(self):
        self.gray_button_flag = False 
        self.white_button_flag = False
        # Initialize the ROS node

        # Create subscribers for the button and joint states
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/phantom/phantom/button', OmniButtonEvent, self.button_event_callback)

        rospy.sleep(0.5)
        
    def button_event_callback(self, data):
        # Process the button event
        self.gray_button = data.grey_button
        self.white_button = data.white_button

        if data.grey_button > 0:
            rospy.loginfo("Gray Button pressed!")
            self.gray_button_flag = not self.gray_button_flag  # Toggle the omni_flag

        
        if data.white_button > 0:
            rospy.loginfo("White Button pressed!")
            self.white_button_flag = not self.white_button_flag

    def joint_states_callback(self, data):
        self.joint_positions = np.array(data.position)


if __name__ == '__main__':
    rospy.init_node('phantom_omni_joint_echo')
    try:
        my_omni = myOmni()  # Initialize the myOmni object
        init_joints = my_omni.joint_positions
        while not rospy.is_shutdown():
            if my_omni.gray_button_flag:
                rospy.loginfo("Phantom Omni Joint States: %s", my_omni.joint_positions)

            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
