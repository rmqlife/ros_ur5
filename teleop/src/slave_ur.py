#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cartesian_state_msgs.msg import PoseTwist
from std_msgs.msg import String

joint_state = np.zeros(6)
joint_goal = np.zeros(6)
robot_control = np.zeros(6)
end_position = np.zeros(7)
singal = True

def robot_joint_callback(robot_joint):
    global singal
    global robot_control

    if singal:
        robot_control = robot_joint.position[:6]
        singal = False

def motion_joint_control(robot_joint, slave_joint):
    global joint_goal

    if all(slave_joint != 0) and all(robot_joint != 0):
        joint_goal = [
            slave_joint[2] + robot_joint[0],
            slave_joint[1] + robot_joint[1],
            slave_joint[0] + robot_joint[2],
            slave_joint[3] + robot_joint[3],
            slave_joint[4] + robot_joint[4],
            slave_joint[5] + robot_joint[5]
        ]

        if all(joint_goal != 0):
            rospy.loginfo("Joint Goals: %s", joint_goal)

            msg = JointTrajectory()
            msg.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            point = JointTrajectoryPoint()
            point.positions = joint_goal
            point.time_from_start = rospy.Duration(1.0)

            msg.points.append(point)
            pub.publish(msg)

def hand_pose_callback(state_msg):
    global joint_state
    joint_state = np.array(state_msg.position[:6])
    motion_joint_control(robot_control, joint_state)

def state_arm_callback(msg):
    global singal
    global end_position

    if singal:
        end_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        singal = False

def main():
    global pub

    rospy.init_node('slave_ur')
    pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.Subscriber('/phantom/phantom/joint_states', JointState, hand_pose_callback)
    rospy.Subscriber('/joint_states', JointState, robot_joint_callback)
    rospy.Subscriber('/topic_arm_state', PoseTwist, state_arm_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
