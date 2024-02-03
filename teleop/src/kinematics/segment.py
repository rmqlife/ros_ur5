import numpy as np
import pickle
import roboticstoolbox as rtb
from rtb import fk, display,center_pose



# Load robot data from a pickle file using RTB
robot = rtb.models.DH.UR5()
with open('data.pkl', 'rb') as f:
    data = pickle.load(f)
    poses = []

# Calculate forward kinematics for each joint configuration
for joints in data:
    pose = fk(joints, robot)
    poses.append(pose)

from simplify import simplify_trajectory
poses = simplify_trajectory(poses, min_curvature=0.2)
display(poses)

# Assuming you already have a center pose
import rtb
center = center_pose(poses) # Replace with actual center coordinates

# Assuming you have a list of poses named 'poses'
# Calculate the squared radius (for efficient comparison)
radius_squared = 0.1**2

# Find all poses within the specified radius
poses_within_radius = []

for pose in poses:
    # Calculate the Euclidean distance squared between the current pose and the center
    distance_squared = sum((pose - center)**2)
    
    if distance_squared <= radius_squared:
        poses_within_radius.append(pose)


display(poses_within_radius)
import pickle
with open('dip.pkl','wb') as f:
    pickle.dump(poses_within_radius,f)