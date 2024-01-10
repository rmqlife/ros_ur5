import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped
from filterpy.kalman import KalmanFilter
import numpy as np
from myOmni import MyBag

class MyFTSensor(MyBag):
    def __init__(self, topic='/sunrise/force'):
        super().__init__(topic, filename="force")
        self.force_sensor = rospy.Subscriber(topic, WrenchStamped, self.subscriber_callback)
        self.force_x_original = []  # List to store original force data
        self.force_x_filtered = []  # List to store filtered force data
        self.kf = KalmanFilter(dim_x=1, dim_z=1)
        self.kf.x = np.array([0.0])  # Initial state estimate
        self.kf.F = np.array([[1]])  # State transition matrix
        self.kf.H = np.array([[1]])  # Measurement matrix
        self.kf.P *= 1000.0  # Initial state covariance matrix (tune as needed)
        self.start_time = rospy.Time.now()

    def subscriber_callback(self, data):
        super().subscriber_callback(data)
        force_x = data.wrench.force.x
        current_time = rospy.Time.now()

        # Calculate time elapsed since the start
        time_elapsed = (current_time - self.start_time).to_sec()

        if time_elapsed <= 1.0:  # Use the first 1 second's data as the base
            pass  # You can choose to do nothing during the first second for baseline
        else:
            if self.force_x_original is None:
                self.force_x_original = [force_x]
            else:
                # Predict and update the Kalman filter
                self.kf.predict()
                self.kf.update(np.array([force_x]))
                filtered_force_x = self.kf.x[0]
                self.force_x_original.append(force_x)

    def plot_force_data(self):
        min_length = min(len(self.force_x_original), len(self.force_x_filtered))
        time = [i for i in range(min_length)]

        plt.figure(figsize=(10, 6))
        plt.plot(time, self.force_x_original[:min_length], label='Original Force X')
        plt.plot(time, self.force_x_filtered[:min_length], label='Filtered Force X', linestyle='--')
        plt.xlabel('Time')
        plt.ylabel('Force X')
        plt.legend()
        plt.grid(True)
        plt.title('Original vs. Filtered Force X')
        plt.show()

if __name__ == "__main__":
    rospy.init_node('my_ft_sensor_test_node')
    my_ft_sensor = MyFTSensor()

    # Let the subscriber collect data for a certain time
    rospy.sleep(3)  # Adjust the time as needed

    my_ft_sensor.plot_force_data()
