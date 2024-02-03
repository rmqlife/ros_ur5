import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from shapely.ops import simplify
from mpl_toolkits.mplot3d import Axes3D

def display_simplified_trajectory(poses, tolerance):
    x_positions, y_positions, z_positions = zip(*poses)  # Unzip the list of poses

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x_positions, y_positions, z_positions, c='b', marker='o', label='TCP Positions')

    # Convert the poses to a LineString
    trajectory_line = LineString(poses)

    # Simplify the LineString using Douglas-Peucker algorithm
    simplified_trajectory = simplify(trajectory_line, tolerance)

    x_simplified, y_simplified, z_simplified = zip(*simplified_trajectory.coords)

    # Create a continuous curve for the simplified trajectory using a blue line
    ax.plot(x_simplified, y_simplified, z_simplified, c='b', label='Simplified Trajectory', linestyle='-')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.title('TCP Positions and Simplified Trajectory')
    plt.legend()
    plt.show()

# Example usage:
poses = [
    (0.0, 0.0, 0.0),
    (1.0, 1.0, 1.0),
    (2.0, 2.0, 2.0),
    (3.0, 3.0, 3.0),
    (4.0, 4.0, 4.0),
    (5.0, 5.0, 5.0)
]

# Set the tolerance level for simplification (adjust as needed)
tolerance = 0.5

display_simplified_trajectory(poses, tolerance)
