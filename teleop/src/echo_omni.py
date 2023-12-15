#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniButtonEvent

class myOmni:
    def __init__(self):
        self.omni_flag = False  # Initialize omni_flag as False

        # Initialize the ROS node
        rospy.init_node('phantom_omni_joint_echo')

        # Create subscribers for the button and joint states
        rospy.Subscriber('/phantom/phantom/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/phantom/phantom/button', OmniButtonEvent, self.button_event_callback)

    def button_event_callback(self, data):
        # Process the button event
        self.gray_button = data.grey_button
        self.white_button = data.white_button

        if data.grey_button > 0:
            rospy.loginfo("Gray Button pressed!")
            self.omni_flag = not self.omni_flag  # Toggle the omni_flag

            if self.omni_flag:
                rospy.loginfo("Omni Flag is now True")  # Log the current status
            else:
                rospy.loginfo("Omni Flag is now False")
        

        if data.white_button > 0:
            rospy.loginfo("White Button pressed!")

    def joint_states_callback(self, data):
        self.joint_positions = np.array(data.position)


if __name__ == '__main__':
    try:
        my_omni = myOmni()  # Initialize the myOmni object
        while not rospy.is_shutdown():
            if my_omni.omni_flag:
                rospy.loginfo("Phantom Omni Joint States: %s", joint_positions)

            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
