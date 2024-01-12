#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np
from reset_moveit import MyRobotMoveit
from myOmni import MyBag

from collections import deque

class MyFTSensor(MyBag):
    def __init__(self, topic='/sunrise/force', buffer_length=50):
        super().__init__(topic, filename="force")
        rospy.Subscriber(topic, WrenchStamped, self.subscriber_callback)
        self.force_data_buffer = deque(maxlen=buffer_length)  # Initialize a rolling buffer to store force data

    def subscriber_callback(self, data):
        super().subscriber_callback(data)
        force_x = data.wrench.force.x
        force_y = data.wrench.force.y
        force_z = data.wrench.force.z
        
        # Store the force values in the rolling buffer along with their timestamps
        current_time = rospy.Time.now()
        self.force_data_buffer.append((current_time, np.array([force_x, force_y, force_z])))

    def get_buffer_time(self):
            # Return the time range in seconds covered by the buffer
            if not self.force_data_buffer:
                return 0.0  # Return 0 seconds if there is no data in the buffer

            oldest_time = self.force_data_buffer[0][0]
            newest_time = self.force_data_buffer[-1][0]
            time_range = (newest_time - oldest_time).to_sec()
            time_range = np.round(time_range, 2)
            return time_range


    def get_force(self):
        # Calculate the average force of the data in the rolling buffer
        if not self.force_data_buffer:
            return None  # Return None if there is no data in the buffer

        total_force = np.sum([data for _, data in self.force_data_buffer], axis=0)

        
        average_force = total_force / len(self.force_data_buffer)
        average_force = np.round(average_force, 2)
        return average_force




def move_z(robot, mag):
    new_pose = robot.get_pose()
    new_pose.position.z += 0.001 * mag  # Adjust the value for your desired movement
    robot.set_trajectory([new_pose])

def detect_contact():
    shake_delta = 0.01  # Adjust the value as needed
    force_threshold = 3.0  # Adjust the force threshold as needed
    stop_duration = 3.0  # Duration to stop the robot when force exceeds threshold

    # Create an instance of MyRobotMoveit
    robot = MyRobotMoveit(group_name="manipulator")
    ftsensor = MyFTSensor()

    stop_flag = False
    direction = 0.1

    ftsensor.start_bag_recording()
    while not rospy.is_shutdown():
        force = ftsensor.get_force()
        if force is not None:
            force_magnitude = np.linalg.norm(force)
            print(force[2])

            if force_magnitude > force_threshold:
                stop_flag = True
                print('Need to stop')
                

            if not stop_flag:
                # Force is small, keep moving (e.g., moving down)
                move_z(robot, mag=-2)

        rospy.sleep(0.01)
    ftsensor.stop_bag_recording()


if __name__ == '__main__':
    try:
        rospy.init_node('ur5_move_with_force_detection', anonymous=True)
        ftsensor = MyFTSensor()
        while not rospy.is_shutdown():
            force = ftsensor.get_force()
            if force is not None:
                # force_magnitude = np.linalg.norm(force)
                print(force, ftsensor.get_buffer_time())
                rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
