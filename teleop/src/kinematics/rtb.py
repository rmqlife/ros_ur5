import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
import pickle

def fk(joints, robot):
    T = robot.fkine(joints)
    position = T.t
    return position

def center_pose(poses):
    if len(poses) == 0:
        return None

    # Convert the list of arrays into a NumPy array for easy calculations
    poses_array = np.array(poses)

    # Calculate the mean along each axis (X, Y, Z) to find the center
    center_position = np.mean(poses_array, axis=0)

    return tuple(center_position)

def display(poses):
    x_positions = [pose[0] for pose in poses]
    y_positions = [pose[1] for pose in poses]
    z_positions = [pose[2] for pose in poses]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x_positions, y_positions, z_positions, c='b', marker='o', label='TCP Positions')
    ax.plot(x_positions, y_positions, z_positions, c='r', label='Trajectory')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.title('TCP Positions and Trajectory')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    robot = rtb.models.DH.UR5()

    # Load data from a pickle file (replace 'your_data.pkl' with the actual file path)
    with open('data.pkl', 'rb') as f:
        data = pickle.load(f)

    poses = []
    for joints in data:
        pose = fk(joints, robot)
        poses.append(pose)

    print(center(poses))
    display(poses)