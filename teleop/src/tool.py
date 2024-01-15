#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def change_digital_output():
    # Initialize a ROS node
    rospy.init_node('ur5_digital_output_control')

    # Create a publisher for the digital_out topic to control the digital output
    digital_out_pub = rospy.Publisher('/ur_driver/digital_out_0', Bool, queue_size=1)

    # Create a Bool message and set its data to False (0)
    digital_out_msg = Bool()
    digital_out_msg.data = True  # Set the digital output to 0 (low)

    # Publish the Bool message to change the digital output to 0
    digital_out_pub.publish(digital_out_msg)

    # Sleep for a short duration (optional)
    rospy.sleep(1)

    # Shutdown the ROS node
    rospy.signal_shutdown('Digital output changed to 0')

if __name__ == '__main__':
    try:
        change_digital_output()
    except rospy.ROSInterruptException:
        pass
