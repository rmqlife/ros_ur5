import numpy as np
import matplotlib.pyplot as plt
import pickle
import roboticstoolbox as rtb

from simplify import simplify_trajectory, compute_curvature
from sklearn.cluster import DBSCAN

def cluster_poses(poses, epsilon):
    # Convert poses to a numpy array for DBSCAN
    poses_array = np.array(poses)

    # Create a DBSCAN object with the specified epsilon (distance threshold)
    dbscan = DBSCAN(eps=epsilon, min_samples=1, metric='euclidean')  # You can adjust min_samples as needed

    # Fit the DBSCAN model to the data and obtain cluster labels
    dbscan.fit(poses_array)

    return dbscan.labels_

def display_labels(poses, labels):
    x_positions, y_positions, z_positions = zip(*poses)  # Unzip the list of poses

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Get unique cluster labels
    unique_labels = np.unique(labels)

    for label in unique_labels:
        cluster_indices = np.where(labels == label)[0]
        cluster_poses = [poses[i] for i in cluster_indices]

        x_cluster, y_cluster, z_cluster = zip(*cluster_poses)

        # Create a continuous curve for each cluster using a unique color
        ax.plot(x_cluster, y_cluster, z_cluster, label=f'Cluster {label}', linestyle='-')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.title('TCP Positions and Clustered Trajectory')
    plt.legend()
    plt.show()


def find_largest_cluster(poses, labels):
    unique_labels, cluster_counts = np.unique(labels, return_counts=True)
    largest_cluster_label = unique_labels[np.argmax(cluster_counts)]
    largest_cluster_indices = np.where(labels == largest_cluster_label)[0]
    largest_cluster_poses = [poses[i] for i in largest_cluster_indices]
    return largest_cluster_poses

if __name__ == "__main__":
    # Load robot data from a pickle file using RTB
    robot = rtb.models.DH.UR5()
    with open('data.pkl', 'rb') as f:
        data = pickle.load(f)
        poses = []

    from rtb import fk,display
    for joints in data:
        pose = fk(joints, robot)
        poses.append(pose)
    # Simplify the trajectory
    
    poses = simplify_trajectory(poses, min_curvature=0.3)
    display(poses)    

    # Cluster the simplified trajectory
    labels = cluster_poses(poses, epsilon=0.5)


    # Display the clustered trajectory with different colors for each cluster
    display_labels(poses, labels)

    
    # poses = find_largest_cluster(poses, labels)

    # display(poses)