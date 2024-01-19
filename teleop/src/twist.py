#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def draw_circle():
    rospy.init_node('circle_drawer')

    # Define the topic name where you send the Twist commands to control the robot
    motion_command_topic = '/your_robot/motion_command_topic'  # Replace with your actual topic name

    # Create a publisher to send Twist commands to control the robot's motion
    motion_publisher = rospy.Publisher(motion_command_topic, Twist, queue_size=10)

    # Set the rate at which to publish Twist commands (adjust as needed)
    rate = rospy.Rate(10)  # 10 Hz

    # Set the parameters for drawing the circle
    radius = 0.2  # Adjust the radius as needed
    angular_speed = 0.5  # Adjust the angular speed as needed

    while not rospy.is_shutdown():
        # Calculate the linear and angular velocities to draw a circle
        linear_velocity = 0.0
        angular_velocity = angular_speed

        # Create a Twist message with the calculated velocities
        cmd_twist = Twist()
        cmd_twist.linear.x = linear_velocity
        cmd_twist.angular.z = angular_velocity

        # Publish the Twist message to control the robot's motion
        motion_publisher.publish(cmd_twist)

        rate.sleep()

if __name__ == '__main__':
    try:
        draw_circle()
    except rospy.ROSInterruptException:
        pass
