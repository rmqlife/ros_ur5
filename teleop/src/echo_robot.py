#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from myRobot import MyRobot
from util import rad2deg, deg2rad
import numpy as np


if __name__ == '__main__':
    try:
        rospy.init_node('ur5_shake_test', anonymous=True)
        record = False
        robot = MyRobot()  # Initialize the robot object
        print(robot.get_joints())
    except rospy.ROSInterruptException:
        pass
