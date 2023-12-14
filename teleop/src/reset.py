#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from util import rad2deg, deg2rad  # Import your rad2deg and deg2rad functions
from shake import publish_joint_trajectory

# reset pose
RESET_POSE = [90, -90, -180, 180, 270, 90]


# Global variables for joint names and positions
joint_names = []
current_joint_positions = []

def robot_joint_callback(data):
    global joint_names, current_joint_positions
    joint_names = data.name
    current_joint_positions = data.position
    pass

if __name__ == '__main__':
    try:
        rospy.init_node('set_robot_position', anonymous=True)
        robot_joint_subscriber = rospy.Subscriber('/joint_states', JointState, robot_joint_callback)

        while not rospy.is_shutdown() and not joint_names:
            rospy.sleep(0.1)

        pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

        if current_joint_positions:
            current_joint_degrees = rad2deg(current_joint_positions)
        print("current robot degrees", current_joint_degrees)

        publish_joint_trajectory(pub, joint_names, deg2rad(current_joint_degrees))
        rospy.sleep(0.5)
        publish_joint_trajectory(pub, joint_names,  deg2rad(RESET_POSE))
        rospy.sleep(0.5)        
    except rospy.ROSInterruptException:
        pass
