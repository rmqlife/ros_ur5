#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped

force_msg = None
# Define the valid force range
MIN_FORCE = -10.0
MAX_FORCE = 10.0

def clip_force(value):
    return np.clip(value, MIN_FORCE, MAX_FORCE)

def force_callback(data):
    global force_msg
    # Create a new WrenchStamped message with only the force values for X, Y, and Z axes
    force_msg = WrenchStamped()
    force_msg.header = data.header

    # Tweaking
    force_msg.wrench.force.x = clip_force(data.wrench.force.y)
    force_msg.wrench.force.y = clip_force(-data.wrench.force.x)
    force_msg.wrench.force.z = clip_force(-data.wrench.force.z)
    
    # Publish the modified message to the "/phantom/phantom/force_feedback" topic
    force_pub.publish(force_msg)

def monitor_output(event):
    # Log the received force values every 0.5 seconds (2Hz)
    rospy.loginfo("Published modified force: (%.2f, %.2f, %.2f)", force_msg.wrench.force.x, force_msg.wrench.force.y, force_msg.wrench.force.z)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('force_feedback_publisher', anonymous=True)

        # Create a subscriber for the "/sunrise/force" topic
        force_sub = rospy.Subscriber('/sunrise/force', WrenchStamped, force_callback)

        # Create a publisher for the "/phantom/phantom/force_feedback" topic
        force_pub = rospy.Publisher('/phantom/phantom/force_feedback', WrenchStamped, queue_size=10)

        # Set up a timer to monitor and log the output at 2Hz
        rospy.Timer(rospy.Duration(0.5), monitor_output, oneshot=False)

        rospy.loginfo("Force feedback publisher node is running.")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
