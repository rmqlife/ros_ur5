#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniButtonEvent
from myOmni import MyOmni

if __name__ == '__main__':
    rospy.init_node('phantom_omni_joint_echo')
    try:
        my_omni = MyOmni()  # Initialize the myOmni object
        while not rospy.is_shutdown():
            if my_omni.gray_button_flag:
                rospy.loginfo("Phantom Omni Joint States: %s", my_omni.get_joints())
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
