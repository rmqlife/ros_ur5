#!/usr/bin/env python3
import rospy
from util import deg2rad,rad2deg,swap_order
from moveit_commander import MoveGroupCommander
from reset import *
import numpy as np

class MyRobotMoveit:
    def __init__(self, group_name="manipulator"):
        self.arm = MoveGroupCommander(group_name)

    def set_joints(self, robot_radians):
        self.arm.set_joint_value_target(robot_radians)
        self.arm.go()

    def get_joints(self):
        current_joint_positions = self.arm.get_current_joint_values()
        return np.array(current_joint_positions)

    def get_pose(self):
        return self.arm.get_current_pose().pose

    def set_trajectory(self, waypoints):
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,
            0.5,
            0.0,
            avoid_collisions=True
        )
        self.arm.execute(plan, wait=False)


if __name__ == '__main__':
    try:
        rospy.init_node('ur5_reset_position', anonymous=True)
        # Create a MoveGroupCommander for the UR5 robot
        robot = MyRobotMoveit()
        robot.arm.clear_trajectory_constraints()
        robot.arm.clear_path_constraints()
        current_joints = robot.get_joints()
        target_joints = RESET_POSE

        print(rad2deg(current_joints))
        print(target_joints)
        # Define the target joint values for reset


        robot.set_joints(deg2rad(target_joints))
        rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        pass
