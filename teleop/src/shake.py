#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from util import rad2deg, deg2rad
import numpy as np

class myRobot:
    def __init__(self):
        # Create a subscriber to the '/joint_states' topic
        self.robot_joint_subscriber = rospy.Subscriber('/joint_states', JointState, self.robot_joint_callback)
        self.pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

        # Initialize joint names and positions
        self.joint_names = []
        self.current_joint_positions = []

        # Wait for the subscriber to receive joint names
        while not rospy.is_shutdown() and not self.joint_names:
            rospy.sleep(0.1)


    def robot_joint_callback(self, data):
        # Store the received joint states in the class variables
        self.joint_names = data.name
        self.joint_positions = np.array(data.position)

    def move_joints(self, joint_positions):
        if not self.joint_names:
            rospy.logwarn("No joint names available yet. Waiting for joint_states message.")
            return

        if len(joint_positions) != len(self.joint_names):
            rospy.logerr("Joint positions do not match the number of joint names.")
            return

        # Create a JointTrajectory message
        joint_traj = JointTrajectory()

        # Set the joint names from the received message
        joint_traj.joint_names = self.joint_names

        # Create a JointTrajectoryPoint for the desired joint positions
        point = JointTrajectoryPoint()
        point.positions = joint_positions

        # Set the time from start for this point (you can adjust this)
        point.time_from_start = rospy.Duration(0.1)  # 0.1 seconds between each point

        # Append the JointTrajectoryPoint to the trajectory
        joint_traj.points.append(point)
        self.pub.publish(joint_traj)  # Add parentheses to call the publish method

if __name__ == '__main__':
    try:
        rospy.init_node('ur5_shake_test', anonymous=True)

        my_robot = myRobot()  # Initialize the robot object
        shake_delta = 1
        
        joint_degrees = rad2deg(my_robot.joint_positions)

        for i in range(len(joint_degrees)):
            for j in [-1, 1]:
                joint_degrees[i] += j * shake_delta
                joint_positions = deg2rad(joint_degrees)
                print('desired rads', joint_degrees)
                my_robot.move_joints(joint_positions)  # Call the move_joints method
                rospy.sleep(0.5)  # Sleep for 0.5 seconds between movements

    except rospy.ROSInterruptException:
        pass
