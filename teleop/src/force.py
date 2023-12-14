#!/usr/bin/env python

import rospy
from sunrise.sriCommDefine import CSRICommManager
from geometry_msgs.msg import WrenchStamped
import time
import sys

WRENCH_TOPIC = "/phantom/phantom/force_feedback"

def main():
    print("SRI TCP Client Demo.")
    
    # Initialize ROS node
    rospy.init_node("wrench_signal_generate")
    n = rospy.NodeHandle()

    # Create a publisher for the wrench topic
    wrench_pub = rospy.Publisher(WRENCH_TOPIC, WrenchStamped, queue_size=5)

    # Initialize CSRICommManager
    commManager = CSRICommManager()

    if commManager.Init():
        if commManager.Run():
            pass

    outfile = open("/home/duan/catkin_ws/output_sensor.txt", "w")

    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        receive_value = commManager.output

        wrench_msg = WrenchStamped()
        wrench_msg.wrench.force.x = receive_value[0]
        wrench_msg.wrench.force.y = receive_value[2]
        wrench_msg.wrench.force.z = receive_value[1]

        print("wrench_msg.wrench.force.x =", wrench_msg.wrench.force.x)
        print("wrench_msg.wrench.force.y =", wrench_msg.wrench.force.y)
        print("wrench_msg.wrench.force.z =", wrench_msg.wrench.force.z)

        wrench_pub.publish(wrench_msg)
        
        rate.sleep()

    outfile.close()
    print("Demo done!\nPress ENTER to close.")
    raw_input()  # Wait for user input to exit

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
