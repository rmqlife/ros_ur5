#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from shake_moveit import MyRobotMoveit
import math

def circle_points(current_pose, radius=0.1, num_points=20):
    waypoints = []
    euler_angles = (0.0, 0.0, 0.0)

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        delta_x = radius * math.cos(theta)
        delta_y = radius * math.sin(theta)

        new_pose = Pose()
        new_pose.position.x = current_pose.position.x
        new_pose.position.y = current_pose.position.y + delta_y
        new_pose.position.z = current_pose.position.z + delta_x
        new_pose.orientation = current_pose.orientation

        waypoints.append(new_pose)

    return waypoints

def rectangle_points(current_pose, length=0.1, width=0.05, num_points=20):
    waypoints = []

    # Extract the current position of the end effector
    x = current_pose.position.x
    y = current_pose.position.y
    z = current_pose.position.z
    orientation = current_pose.orientation

    # Compute the corner points of the rectangle
    corners = [
        (x + length / 2, y + width / 2),
        (x + length / 2, y - width / 2),
        (x - length / 2, y - width / 2),
        (x - length / 2, y + width / 2),
    ]

    for i in range(num_points):
        # Interpolate between the corner points to create the rectangle waypoints
        t = float(i) / float(num_points - 1)
        x_interpolated = (1 - t) * corners[0][0] + t * corners[2][0]
        y_interpolated = (1 - t) * corners[0][1] + t * corners[2][1]

        new_pose = Pose()
        new_pose.position.x = x_interpolated
        new_pose.position.y = y_interpolated
        new_pose.position.z = z
        new_pose.orientation = orientation

        waypoints.append(new_pose)

    return waypoints

def main():
    try:
        rospy.init_node('draw_circle_moveit', anonymous=True)
        arm = MyRobotMoveit()
        current_pose = arm.get_pose()
        waypoints = circle_points(current_pose)
        waypoints += waypoints 
        waypoints.append(current_pose)
        arm.set_trajectory(waypoints)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

