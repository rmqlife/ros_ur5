#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time
from reset_moveit import MyRobotMoveit
from myOmni import MyBag

import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from filterpy.kalman import KalmanFilter
class MyFTSensor(MyBag):
    def __init__(self, topic='/sunrise/force'):
        super().__init__(topic, filename="force")
        self.force_sensor = rospy.Subscriber(topic, WrenchStamped, self.subscriber_callback)
        self.force = None
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.x = np.array([0.0, 0.0, 0.0])  # Initial state estimate
        self.kf.F = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # State transition matrix
        self.kf.H = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # Measurement matrix
        self.kf.P *= 1000.0  # Initial state covariance matrix (tune as needed)
        self.start_time = rospy.Time.now()

    def subscriber_callback(self, data):
        super().subscriber_callback(data)
        force_x = data.wrench.force.x
        force_y = data.wrench.force.y
        force_z = data.wrench.force.z
        current_time = rospy.Time.now()

        # Calculate time elapsed since the start
        time_elapsed = (current_time - self.start_time).to_sec()

        if time_elapsed <= 1.0:  # Use the first 1 second's data as the base
            pass  # You can choose to do nothing during the first second for baseline
        else:
            if self.force is None:
                self.force = np.array([force_x, force_y, force_z])
            else:
                # Predict and update the Kalman filter
                self.kf.predict()
                self.kf.update(np.array([force_x, force_y, force_z]))
                self.force = self.kf.x

    def get_force(self):
        return self.force


def move_z(robot, mag):
    new_pose = robot.get_pose()
    new_pose.position.z += 0.001 * mag  # Adjust the value for your desired movement
    robot.set_trajectory([new_pose])

if __name__ == '__main__':
    try:
        rospy.init_node('ur5_move_with_force_detection', anonymous=True)
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
    except rospy.ROSInterruptException:
        pass
