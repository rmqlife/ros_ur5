#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from moveit_commander import MoveGroupCommander
from util import rad2deg, deg2rad
import numpy as np

# Global variable to store the force values
force = None

def force_callback(data):
    global force
    # Extract force values for X, Y, and Z axes
    force_x = data.wrench.force.x
    force_y = data.wrench.force.y
    force_z = data.wrench.force.z
    force = [force_x, force_y, force_z]

def set_robot_pose(arm, pose):
    arm.set_pose_target(pose)
    arm.go()

def get_robot_pose(arm):
    # Get the current end-effector pose
    current_pose = arm.get_current_pose().pose
    return current_pose

if __name__ == '__main__':
    try:
        rospy.init_node('ur5_reset_position', anonymous=True)
        # Initialize MoveIt! commander
        shake_delta = 0.01  # Adjust the value as needed

        # Create a MoveGroupCommander for the UR5 robot
        arm = MoveGroupCommander("manipulator")

        # Create a subscriber for the "/phantom/phantom/force_feedback" topic
        force_sub = rospy.Subscriber('/phantom/phantom/force_feedback', WrenchStamped, force_callback)

        # Define the target pose incrementally based on force feedback
        init_pose = get_robot_pose(arm)

        while not rospy.is_shutdown():
            if force is not None:
                force_magnitude = np.linalg.norm(force)

                if force_magnitude > 2:
                    current_pose = get_robot_pose(arm)
                    force_vector = np.array(force)
                    # Adjust the pose based on the force feedback (you can modify this logic)
                    pose_adjustment = shake_delta * force_vector
                    new_pose = init_pose
                    new_pose.position.x = current_pose.position.x + pose_adjustment[1]
                    new_pose.position.y = current_pose.position.y - pose_adjustment[0]
                    new_pose.position.z = current_pose.position.z + pose_adjustment[2]
                    
                    set_robot_pose(arm, new_pose)
            rospy.sleep(0.02)
            
    except rospy.ROSInterruptException:
        pass
