#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState

def joint_states_callback(msg):
    joint_values = np.array(msg.position)
    rospy.loginfo("Phantom Omni Joint States: %s", joint_values)

def main():
    rospy.init_node('phantom_omni_joint_echo')
    rospy.Subscriber('/phantom/phantom/joint_states', JointState, joint_states_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
