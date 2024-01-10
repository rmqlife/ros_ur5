#!/usr/bin/env python3
import rospy
from util import deg2rad,rad2deg,swap_order
from moveit_commander import MoveGroupCommander
from reset import RESET_POSE_RAD
import numpy as np

RESET_POSE = swap_order(RESET_POSE_RAD, 0, 2)

class MyRobotMoveit:
    def __init__(self, group_name="manipulator"):
        self.arm = MoveGroupCommander(group_name)

    def move_joints(self, robot_radians):
        self.arm.set_joint_value_target(robot_radians)
        self.arm.go()

    def get_joints(self):
        return np.array(self.arm.get_current_joint_values())

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

        print(current_joints)
        print(RESET_POSE)
        # Define the target joint values for reset

        robot.move_joints(RESET_POSE)
        rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        pass
