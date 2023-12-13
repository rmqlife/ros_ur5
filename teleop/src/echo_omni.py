#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniButtonEvent

omni_flag=False;

def button_event_callback(data):
    global omni_flag;
    # Process the button event
    if data.grey_button>0:
        rospy.loginfo("Gray Button pressed!")
        omni_flag=not omni_flag; #True

    if data.white_button>0:
        rospy.loginfo("White Button pressed!")


def joint_states_callback(data):
    joint_values = np.array(data.position)
    if omni_flag:
        rospy.loginfo("Phantom Omni Joint States: %s", joint_values)


if __name__ == '__main__':
    rospy.init_node('phantom_omni_joint_echo')
    rospy.Subscriber('/phantom/phantom/joint_states', JointState, joint_states_callback)
    rospy.Subscriber('/phantom/phantom/button', OmniButtonEvent, button_event_callback)
    

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
