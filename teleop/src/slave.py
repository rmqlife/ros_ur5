#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from cartesian_state_msgs.msg import PoseTwist

# Global variables
joint_state = [0.0] * 6
joint_values = [0.0] * 6
robot_control = [0.0] * 6
joint_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
singal = True

end_position = [0.0] * 8
last_publish_time = rospy.Time()  # Define last_publish_time as a global variable

def robotjointcallback(robot_joint):
    global singal
    if singal:
        robot_control[0] = robot_joint.position[0]
        robot_control[1] = robot_joint.position[1]
        robot_control[2] = robot_joint.position[2]
        robot_control[3] = robot_joint.position[3]
        robot_control[4] = robot_joint.position[4]
        robot_control[5] = robot_joint.position[5]
        singal = False

def motionjointcontrol(robot_joint, slave_joint):
    init_success = False
    rospy.loginfo("move_group_start")

    for i in range(6):
        if slave_joint[i] != 0 and robot_joint[i] != 0:
            init_success = True

    if init_success:
        joint_goal[0] = slave_joint[2] + robot_joint[0]
        joint_goal[1] = slave_joint[1] + robot_joint[1]
        joint_goal[2] = slave_joint[0] + robot_joint[2]
        joint_goal[3] = slave_joint[3] + robot_joint[3]
        joint_goal[4] = slave_joint[4] + robot_joint[4]
        joint_goal[5] = slave_joint[5] + robot_joint[5]

        if (joint_goal[0] != 0 and joint_goal[1] != 0 and joint_goal[2] != 0 and
            joint_goal[3] != 0 and joint_goal[4] != 0 and joint_goal[5] != 0):

            rospy.loginfo("joint_goal[0]: %f", joint_goal[0])
            rospy.loginfo("joint_goal[1]: %f", joint_goal[1])
            rospy.loginfo("joint_goal[2]: %f", joint_goal[2])
            rospy.loginfo("joint_goal[3]: %f", joint_goal[3])
            rospy.loginfo("joint_goal[4]: %f", joint_goal[4])
            rospy.loginfo("joint_goal[5]: %f", joint_goal[5])

            point = JointTrajectoryPoint()
            point.positions.extend(joint_goal)
            msg = JointTrajectory()
            msg.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            current_time = rospy.Time.now()
            time_interval = current_time - last_publish_time
            point.time_from_start = rospy.Duration(1.0, 0)
            msg.points.append(point)
            pub.publish(msg)

def handposeCallback(state_msg):
    global joint_state
    joint_state[0] = state_msg.position[0]
    joint_state[1] = state_msg.position[1] - 0.26889
    joint_state[2] = state_msg.position[2] + 0.639702
    joint_state[3] = state_msg.position[3] - 3.14276
    joint_state[4] = state_msg.position[4] + 3.15091
    joint_state[5] = state_msg.position[5] + 3.41783
    rospy.loginfo("joint_state[0]: %f", joint_state[0])
    rospy.loginfo("joint_state[1]: %f", joint_state[1])
    rospy.loginfo("joint_state[2]: %f", joint_state[2])
    rospy.loginfo("joint_state[3]: %f", joint_state[3])
    rospy.loginfo("joint_state[4]: %f", joint_state[4])
    rospy.loginfo("joint_state[5]: %f", joint_state[5])
    motionjointcontrol(robot_control, joint_state)

def state_arm_callback(msg):
    global singal
    if singal:
        end_position[0] = msg.pose.position.x
        end_position[1] = msg.pose.position.y
        end_position[2] = msg.pose.position.z
        end_position[3] = msg.pose.orientation.x
        end_position[4] = msg.pose.orientation.y
        end_position[5] = msg.pose.orientation.z
        end_position[6] = msg.pose.orientation.w
        singal = False
    rospy.loginfo("pose x: %f", msg.pose.position.x)
    rospy.loginfo("pose y: %f", msg.pose.position.y)
    rospy.loginfo("pose z: %f", msg.pose.position.z)
    rospy.loginfo("pose_orientation x: %f", msg.pose.orientation.x)
    rospy.loginfo("pose_orientation y: %f", msg.pose.orientation.y)
    rospy.loginfo("pose_orientation z: %f", msg.pose.orientation.z)
    rospy.loginfo("pose_orientation w: %f", msg.pose.orientation.w)

def main():
    rospy.init_node('ur_joint_publisher')
    pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
    hand_pose_subscriber = rospy.Subscriber('/phantom/phantom/joint_states', JointState, handposeCallback)
    robot_jonit_subsciber = rospy.Subscriber('/joint_states', JointState, robotjointcallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
