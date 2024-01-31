#!/usr/bin/env python3
import rospy
from util import deg2rad,rad2deg,swap_order
import numpy as np
from myRobotMoveit import MyRobotMoveit

if __name__ == '__main__':
    try:
        rospy.init_node('ur5_reset_position', anonymous=True)
        # Create a MoveGroupCommander for the UR5 robot
        robot = MyRobotMoveit()
        robot.arm.clear_trajectory_constraints()
        robot.arm.clear_path_constraints()
        current_joints = robot.get_joints()

        print(current_joints)

        from myConfig import MyConfig
        config = MyConfig('../joint_configs.json')
        reset_pose = config.get('reset')

        print(reset_pose)
        # Define the target joint values for reset

        robot.move_joints(reset_pose)
        rospy.sleep(0.5)

    except rospy.ROSInterruptException:
        pass
