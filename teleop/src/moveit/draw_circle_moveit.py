#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from shake_moveit import MyRobotMoveit
import math

import math
import numpy as np
from geometry_msgs.msg import Pose

def quaternion_from_vector_to_z(vector):
    """
    Calculate the quaternion required to rotate the z-axis to align with a given vector.
    :param vector: The target vector.
    :return: Quaternion representing the rotation.
    """
    vector = np.array(vector)
    vector /= np.linalg.norm(vector)
    rotation_axis = np.cross(np.array([0, 0, -1]), vector)
    rotation_angle = np.arccos(np.dot(np.array([0, 0, -1]), vector))
    s = np.sin(rotation_angle / 2)
    return [rotation_axis[0] * s, rotation_axis[1] * s, rotation_axis[2] * s, np.cos(rotation_angle / 2)]


def circle_points(current_pose, radius=0.1, num_points=20):
    waypoints = []
    euler_angles = (0.0, 0.0, 0.0)
    target_position = np.array([0, 0, -40])  # Target position to orient towards

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        delta_x = radius * math.cos(theta)
        delta_y = radius * math.sin(theta)

        new_pose = Pose()
        new_pose.position.x = current_pose.position.x + delta_x
        new_pose.position.y = current_pose.position.y + delta_y
        new_pose.position.z = current_pose.position.z 

        # Calculate the direction vector towards the target position
        direction_vector = target_position - np.array([new_pose.position.x, new_pose.position.y, new_pose.position.z])
        # Calculate the quaternion to orient towards the target position
        orientation = quaternion_from_vector_to_z(direction_vector)
        # Set the orientation of the new pose
        new_pose.orientation.x = orientation[0]
        new_pose.orientation.y = orientation[1]
        new_pose.orientation.z = orientation[2]
        new_pose.orientation.w = orientation[3]

        waypoints.append(new_pose)

    return waypoints


def main():
    try:
        rospy.init_node('draw_circle_moveit', anonymous=True)
        arm = MyRobotMoveit()
        current_pose = arm.get_pose()
        waypoints = circle_points(current_pose)
        waypoints.append(current_pose)
        arm.set_trajectory(waypoints)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

