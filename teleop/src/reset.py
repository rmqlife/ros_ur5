#!/usr/bin/env python3
import rospy, sys
from myRobot import MyRobot
from myConfig import MyConfig # Import the JointConfig class

if __name__ == '__main__':
    try:
        rospy.init_node('reset', anonymous=True)
        my_robot = MyRobot()  # Initialize the robot object

        # Create a JointConfig instance
        joint_configs = MyConfig()

        if len(sys.argv) > 1:
            config_name = sys.argv[1]
        else:
            config_name = 'reset'


        # Check if the reset pose is already saved, and load it if available
        if joint_configs.get(config_name):
            reset_pose = joint_configs.get(config_name)
            print("Loaded reset pose:", reset_pose)
            my_robot.move_joints(reset_pose, duration=2)
        
    except rospy.ROSInterruptException:
        pass
