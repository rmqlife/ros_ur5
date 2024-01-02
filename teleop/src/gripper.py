#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def control_gripper(gripper_position):
    pub = rospy.Publisher('/ur5/gripper_controller/command', Float64, queue_size=10)
    rospy.init_node('ur5_gripper_control', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz (adjust as needed)

    while not rospy.is_shutdown():
        pub.publish(gripper_position)
        rate.sleep()

if __name__ == '__main__':
    try:
        gripper_position = 0.0  # Set the initial gripper position
        control_gripper(gripper_position)
    except rospy.ROSInterruptException:
        pass
