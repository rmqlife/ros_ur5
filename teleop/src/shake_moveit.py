#!/usr/bin/env python3
import rospy
from util import deg2rad, rad2deg
import numpy as np
from reset_moveit import MyRobotMoveit

if __name__ == '__main__':
    try:
        rospy.init_node('ur5_reset_position', anonymous=True)
        shake_delta = 1
        robot = MyRobotMoveit("manipulator")

        robot_radians = robot.get_joints()
        robot_degrees = rad2deg(robot_radians)
        print(robot_degrees)

        if True:
            for i in range(len(robot_radians)):
                for j in [-1, 1]:
                    robot_degrees[i] += j * shake_delta
                    robot.set_joints(deg2rad(robot_degrees))
                rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
