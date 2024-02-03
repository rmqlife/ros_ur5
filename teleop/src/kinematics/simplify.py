import numpy as np
import pickle
import roboticstoolbox as rtb
from scipy.spatial.distance import euclidean

def compute_curvature(poses):
    num_points = len(poses)
    curvature = np.zeros(num_points)
    
    for i in range(1, num_points - 1):
        prev_point = np.array(poses[i - 1])
        current_point = np.array(poses[i])
        next_point = np.array(poses[i + 1])
        
        vector1 = current_point - prev_point
        vector2 = next_point - current_point
        
        cross_product = np.cross(vector1, vector2)
        numerator = np.linalg.norm(cross_product)
        denominator = np.linalg.norm(vector1) * np.linalg.norm(vector2)
        
        curvature[i] = numerator / denominator
    
    return curvature

def simplify_trajectory(poses, min_curvature):
    # Compute the curvature of each point
    curvature = compute_curvature(poses)
    
    # Identify the points with curvature above the threshold
    valid_indices = np.where(curvature >= min_curvature)[0]
    simplified_poses = [poses[i] for i in valid_indices]
    
    return simplified_poses


if __name__ == "__main__":
    # Load robot data from a pickle file using RTB
    robot = rtb.models.DH.UR5()
    with open('data.pkl', 'rb') as f:
        data = pickle.load(f)
        poses = []

    from rtb import fk, display
    for joints in data:
        pose = fk(joints, robot)
        poses.append(pose)

    # Simplify the trajectory
    poses = simplify_trajectory(poses, min_curvature=0.3)

    display(poses)

