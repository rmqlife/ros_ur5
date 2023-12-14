#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import math

def draw_circle(radius=0.05, num_points=20):
    # Initialize the ROS node
    rospy.init_node('draw_circle_without_moveit', anonymous=True)
    # Create a publisher to send poses
    pose_publisher = rospy.Publisher('/target_pose', Pose, queue_size=10)
    rate = rospy.Rate(10)  # Set the publishing rate to 10 Hz

    # Get the current TCP pose
    current_pose = Pose()
    current_pose.position.x = 0.0  # Set your initial position here
    current_pose.position.y = 0.0
    current_pose.position.z = 0.0

    for i in range(num_points):
        # Compute the next pose in the circle
        theta = 2.0 * math.pi * i / num_points
        delta_x = radius * math.cos(theta)
        delta_y = radius * math.sin(theta)

        # Create a new pose
        new_pose = Pose()
        new_pose.position.x = current_pose.position.x + delta_x
        new_pose.position.y = current_pose.position.y + delta_y
        new_pose.position.z = current_pose.position.z

        # Publish the new pose
        pose_publisher.publish(new_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        draw_circle(radius=0.1)  # Adjust the radius as needed
    except rospy.ROSInterruptException:
        pass
