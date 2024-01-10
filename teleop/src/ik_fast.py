from ur_ikfast import ur_kinematics

from util import swap_order
from reset import RESET_POSE_RAD
arm = ur_kinematics.URKinematics('ur5')

joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.]  # in radians

joint_angles = [1.5707963267948966, -1.5707963267948966, -3.141592653589793, 4.71238898038469, 4.71238898038469, 3.141592653589793]
joint_angles = swap_order(joint_angles, 0, 2)
print("joint angles", joint_angles)

pose_quat = arm.forward(joint_angles)
pose_matrix = arm.forward(joint_angles, 'matrix')

print("forward() quaternion \n", pose_quat)
print("forward() matrix \n", pose_matrix)

# print("inverse() all", ur3e_arm.inverse(pose_quat, True))
print("inverse() one from quat", arm.inverse(pose_quat, False, q_guess=joint_angles))

print("inverse() one from matrix", arm.inverse(pose_matrix, False, q_guess=joint_angles))

