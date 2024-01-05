#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from util import rad2deg, deg2rad, swap_order
import numpy as np
from myOmni import MyBag  # Import MyBag class
import sys

class MyRobot(MyBag):  # Inherit from MyBag
    def __init__(self):
        self.topic = '/joint_states'  # Define the topic for reading joint states
        super().__init__(self.topic, filename="ur_joints")  # Initialize the MyBag base class

        # Create a subscriber to the '/joint_states' topic
        self.robot_joint_subscriber = rospy.Subscriber(self.topic, JointState, self.subscriber_callback)
        self.pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

        # Initialize joint names and positions
        self.joint_names = []
        self.current_joint_positions = []

        # Wait for the subscriber to receive joint names
        while not rospy.is_shutdown() and not self.joint_names:
            rospy.sleep(0.1)
        rospy.sleep(0.5)
        
    def subscriber_callback(self, data):
        
        super().subscriber_callback(data)

        self.joint_names = data.name
        self.joint_positions = np.array(data.position)

    def get_joints(self):
        return self.joint_positions
    
    def move_joints(self, joint_positions, duration=0.1):
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

        # Set the time from start for this point
        point.time_from_start = rospy.Duration(duration)

        # Append the JointTrajectoryPoint to the trajectory
        joint_traj.points.append(point)
        self.pub.publish(joint_traj)  # Call the publish method



if __name__ == '__main__':
    try:
        rospy.init_node('ur5_shake_test', anonymous=True)
        from myBagReader import MyBagReader
        bag_reader = MyBagReader(sys.argv[1])
        
        time_interval = 0.1
        data = bag_reader.sample_attribute(topic='/joint_states', attribute='position', time_interval=time_interval)
        
        robot = MyRobot()
        
        for i, joints in enumerate(data):
            print("moving to", joints)
            if i==0: 
                duration=1
            else:
                duration=time_interval
            robot.move_joints(joints, duration=duration)
            print('now at', robot.get_joints())

    except rospy.ROSInterruptException:
        pass

