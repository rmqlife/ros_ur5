#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from util import rad2deg, deg2rad  # Import your rad2deg and deg2rad functions

# Global variables for joint names and positions
joint_names = []
current_joint_positions = []

def robot_joint_callback(data):
    global joint_names, current_joint_positions
    joint_names = data.name
    current_joint_positions = data.position

def publish_joint_trajectory(pub, joint_positions):
    if not joint_names:
        rospy.logwarn("No joint names available yet. Waiting for joint_states message.")
        return

    if len(joint_positions) != len(joint_names):
        rospy.logerr("Joint positions do not match the number of joint names.")
        return

    joint_traj = JointTrajectory()
    joint_traj.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(0.1)  # Adjust time as needed

    joint_traj.points.append(point)
    pub.publish(joint_traj)

def set_robot_position(pub, joint_positions):
    rospy.sleep(0.5)  # Sleep for 0.5 seconds between movements
    publish_joint_trajectory(pub, deg2rad(joint_positions))
    rospy.sleep(0.5)  # Sleep for 0.5 seconds between movements

if __name__ == '__main__':
    try:
        rospy.init_node('set_robot_position', anonymous=True)
        robot_joint_subscriber = rospy.Subscriber('/joint_states', JointState, robot_joint_callback)

        while not rospy.is_shutdown() and not joint_names:
            rospy.sleep(0.1)

        pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

        if current_joint_positions:
            current_joint_degrees = rad2deg(current_joint_positions)
        
        print('Current position in degrees:', current_joint_degrees)
        set_robot_position(pub, current_joint_degrees)
        desired_joint_degrees = [90, -90, -180, 180, 270, 90]
        print('Desired position in degrees:', desired_joint_degrees)
        set_robot_position(pub, desired_joint_degrees)

    except rospy.ROSInterruptException:
        pass
