#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import sys
from util import rad2deg, deg2rad

# Global variable to store joint names and positions
joint_names = []
current_joint_positions = []

def robot_joint_callback(data):
    # Store the received joint states in the global variables
    global joint_names, current_joint_positions
    joint_names = data.name
    current_joint_positions = data.position

def init_robot_pub():
    return rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

def to_joint_trajectory(joint_names, joint_positions):
    if not joint_names:
        rospy.logwarn("No joint names available yet. Waiting for joint_states message.")
        return

    if len(joint_positions) != len(joint_names):
        rospy.logerr("Joint positions do not match the number of joint names.")
        return

    # Create a JointTrajectory message
    joint_traj = JointTrajectory()

    # Set the joint names from the received message
    joint_traj.joint_names = joint_names

    # Create a JointTrajectoryPoint for the desired joint positions
    point = JointTrajectoryPoint()
    point.positions = joint_positions

    # Set the time from start for this point (you can adjust this)
    point.time_from_start = rospy.Duration(0.1)  # 0.1 seconds between each point

    # Append the JointTrajectoryPoint to the trajectory
    joint_traj.points.append(point)
    return joint_traj

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('ur5_shake', anonymous=True)

        # Create a subscriber to the '/joint_states' topic
        robot_joint_subscriber = rospy.Subscriber('/joint_states', JointState, robot_joint_callback)

        # Wait for the subscriber to receive joint names
        while not rospy.is_shutdown() and not joint_names:
            rospy.sleep(0.1)

        # Create a publisher for the '/scaled_pos_joint_traj_controller/command' topic
        pub = init_robot_pub()

        shake_delta = 1

        if current_joint_positions:
            current_joint_degrees = rad2deg(current_joint_positions)

        print('now at', current_joint_degrees)
        joint_degrees = current_joint_degrees

        for i in range(len(joint_degrees)):
            for j in [-1, 1]:
                joint_degrees[i] += j * shake_delta
                joint_positions = deg2rad(joint_degrees)
                print('desired rads', joint_degrees)
                pub.publish(to_joint_trajectory(joint_names, joint_positions))
                rospy.sleep(0.5)  # Sleep for 0.1 seconds between movements

    except rospy.ROSInterruptException:
        pass
