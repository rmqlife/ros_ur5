#!/usr/bin/env python3
import rospy
from moveit_commander import MoveGroupCommander
import numpy as np

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
        #  Configurations are computed for every eef_step meters; 
        #  The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath;
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints=waypoints,
            eef_step=0.1,
            jump_threshold=0,
            avoid_collisions=True
        )
        self.arm.execute(plan, wait=True)


# maybe we can use it as a moveit status publisher, by use it to control the robot
if __name__ == '__main__':
    try:
        rospy.init_node('echo_robot', anonymous=True)
        # Initialize the ROS node
        robot = MyRobotMoveit()
        # Create a subscriber to the '/joint_states' topic
        current_joints = robot.get_joints()
        print(current_joints)
        # Set the loop rate to 10Hz
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Call the print_joint_status function at 10Hz
            print("joints", robot.get_joints())
            print("pose", robot.get_pose())
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
