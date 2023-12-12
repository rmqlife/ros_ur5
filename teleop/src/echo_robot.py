#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from util import rad2deg, deg2rad

# Global variable to store joint values in radians
current_joint_positions_rad = []

def robot_joint_callback(data):
    # Store the received joint states in the global variable
    global current_joint_positions_rad
    current_joint_positions_rad = data.position

def print_joint_status():
    # Access the joint values stored in radians
    global current_joint_positions_rad
    if current_joint_positions_rad:
        print("Joint Values in Radians:", current_joint_positions_rad)
        # Convert and print joint values in degrees
        joint_values_deg = rad2deg(current_joint_positions_rad)
        print("Joint Values in Degrees:", joint_values_deg)
    else:
        print("No joint values available yet.")

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('robot_joint_subscriber', anonymous=True)

        # Create a subscriber to the '/joint_states' topic
        robot_joint_subscriber = rospy.Subscriber('/joint_states', JointState, robot_joint_callback)

        # Set the loop rate to 10Hz
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Call the print_joint_status function at 10Hz
            print_joint_status()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
