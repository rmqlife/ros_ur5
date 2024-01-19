#!/usr/bin/env python3
import rospy
from reset_moveit import MyRobotMoveit
from util import rad2deg, deg2rad

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
