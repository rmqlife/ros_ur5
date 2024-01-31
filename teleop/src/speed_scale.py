#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def speed_scale_callback(msg):
    # This callback function will be called when a message is received on the speed scaling topic
    rospy.loginfo("Received speed scale value: %.2f", msg.data)
    rospy.signal_shutdown("Received speed scale value")

def listen_to_speed_scale_once():
    rospy.init_node('speed_scale_listener_once')

    # Define the topic name for reading the speed scale
    speed_scale_topic = '/speed_scaling_factor'

    # Create a subscriber to listen to the speed scaling topic
    rospy.Subscriber(speed_scale_topic, Float64, speed_scale_callback)

    # Spin until we receive a message
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        listen_to_speed_scale_once()
    except rospy.ROSInterruptException:
        pass
