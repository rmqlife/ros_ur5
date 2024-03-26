import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def circle_points(current_pose, radius=0.1, num_points=20):
    waypoints = []

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        delta_x = radius * math.cos(theta)
        delta_y = radius * math.sin(theta)

        waypoint_vector = np.array([current_pose[0] + delta_x, current_pose[1] + delta_y, current_pose[2]])
        waypoints.append(waypoint_vector)

    return waypoints

def append_orientation(waypoints, orientation=np.array([0, 0, 0])):
    for i in range(len(waypoints)):
        waypoints[i] = np.concatenate((waypoints[i], orientation))  # Concatenate orientation to waypoints
    return waypoints

def towards_orientation(waypoints, target):
    for waypoint in waypoints:
        orientation = target - waypoint[:3]  # Calculate orientation towards the target
        orientation /= np.linalg.norm(orientation)  # Normalize the orientation vector
        waypoint[3:] = orientation  # Update the orientation of the waypoint
    return waypoints

def draw_waypoints(waypoints, direction_length):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for waypoint in waypoints:
        position = waypoint[:3]
        orientation = waypoint[3:]

        # Scale orientation by direction_length
        orientation *= direction_length

        ax.scatter(position[0], position[1], position[2], color='b')
        ax.quiver(position[0], position[1], position[2], orientation[0], orientation[1], orientation[2], color='g')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Waypoints Visualization')
    plt.show()

# Example usage:
current_pose = np.array([1.0, 2.0, 3.0])  # Current position
radius = 1
num_points = 20
direction_length = 0.1  # Length to scale the orientation vectors
target = np.array([1.0, 2.0, 2.5])  # Target position

waypoints = circle_points(current_pose, radius, num_points)
waypoints_with_orientation = append_orientation(waypoints)
waypoints_with_target_orientation = towards_orientation(waypoints_with_orientation, target)
draw_waypoints(waypoints_with_target_orientation, direction_length)
