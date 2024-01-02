#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point
from shake_moveit import MyRobotMoveit
import math

def circle_points(current_pose, radius=0.05, num_points=20):
    waypoints = []
    euler_angles = (0.0, 0.0, 0.0)

    for i in range(num_points):
        theta = 2.0 * math.pi * i / num_points
        delta_x = radius * math.cos(theta)
        delta_y = radius * math.sin(theta)

        new_pose = Pose()
        new_pose.position.x = current_pose.position.x
        new_pose.position.y = current_pose.position.y + delta_y
        new_pose.position.z = current_pose.position.z + delta_x
        new_pose.orientation = current_pose.orientation

        waypoints.append(new_pose)

    return waypoints

def main():
    try:
        rospy.init_node('draw_circle_moveit', anonymous=True)
        
        arm = MyRobotMoveit()
        current_pose = arm.get_pose()
        waypoints = circle_points(current_pose, radius=0.1)
        waypoints.append(current_pose)
        arm.set_trajectory(waypoints)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

