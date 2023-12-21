#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point
import math  # Import the math module for mathematical operations

def draw_circle(arm, radius=0.05, num_points=20):
    # Get the current TCP pose
    current_pose = arm.get_current_pose().pose
    init_pose = current_pose

    waypoints = []
    euler_angles = (0.0, 0.0, 0.0)  # Roll, Pitch, Yaw (no rotation)

    for i in range(num_points):
        # Compute the next pose in the circle
        theta = 2.0 * math.pi * i / num_points  # Use math.pi for pi value
        delta_x = radius * math.cos(theta)  # Use math.cos for cosine
        delta_y = radius * math.sin(theta)  # Use math.sin for sine

        # Create a new waypoint
        new_pose = Pose()
        new_pose.position.x = current_pose.position.x 
        new_pose.position.y = current_pose.position.y + delta_y
        new_pose.position.z = current_pose.position.z + delta_x
        new_pose.orientation = current_pose.orientation

        waypoints.append(new_pose)

    waypoints.append(init_pose)

    # Plan and execute the trajectory
    (plan, fraction) = arm.compute_cartesian_path(
        waypoints,   # waypoints to follow
        0.5,        # eef_step
        0.0,         # jump_threshold
        avoid_collisions=True  # collision avoidance
    )

    arm.execute(plan, wait=False)

def main():
    try:
        # Initialize the ROS node
        rospy.init_node('draw_circle_moveit', anonymous=True)
        # Create a MoveGroupCommander for the robot arm
        arm = moveit_commander.MoveGroupCommander("manipulator")

        # Set the planning time
        arm.set_planning_time(10.0)
        # Allow re-planning to increase the chances of success
        arm.allow_replanning(True)

        # Draw a circle around the current TCP position
        draw_circle(arm, radius=0.1)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
