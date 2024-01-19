import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped
import numpy as np
from myOmni import MyBag

def array2force(array):
    # Convert a numpy array to a force message
    force_msg = WrenchStamped()
    force_msg.wrench.force.x = array[0]
    force_msg.wrench.force.y = array[1]
    force_msg.wrench.force.z = array[2]
    return force_msg

def force2array(force_msg):
    # Convert a force message to a numpy array
    force_array = np.array([
        force_msg.x,
        force_msg.y,
        force_msg.z
    ])
    return force_array

class MyFTSensor(MyBag):
    def __init__(self, topic='/wrench'):
        super().__init__(topic, filename="force")
        self.force_subscriber = rospy.Subscriber(topic, WrenchStamped, self.subscriber_callback)
        self.force_publisher = rospy.Publisher('/phantom/phantom/force_feedback', WrenchStamped, queue_size=10)

        self.start_time = rospy.Time.now()
        self.omni_flag = True
        self.force_history = []  # Store the received force data
        self.force_bias = None  # Initialize the bias to None

    def subscriber_callback(self, data):
        super().subscriber_callback(data)
        self.force = data.wrench.force
        self.force_history.append(force2array(self.force))

        # # Check the bag if the running time is over 1 second
        if self.force_bias is None and rospy.Time.now() - self.start_time > rospy.Duration(1.0):
            # Compute the bias as the mean of 1 second of data
            force_array = np.stack(self.force_history)
            self.force_bias = np.mean(force_array, axis=0)
            print("Bias:", self.force_bias)  # Corrected variable name

        if self.omni_flag:
            # Apply a factor (0.1 in this case)
            f = force2array(self.force)
            if self.force_bias is not None:
                f = f - self.force_bias
            modified_force = 0.1 * f  # Apply the factor

            # Convert the modified force to a force message
            self.modified_force_msg = array2force(modified_force)
            self.modified_force_msg.wrench.force.z = - self.modified_force_msg.wrench.force.z
            # Publish the modified force message
            self.force_publisher.publish(self.modified_force_msg)

    def plot_force_data(self):
        # Plot the received force data
        force_data = np.array(self.force_history)
        plt.figure()
        plt.plot(force_data)
        plt.xlabel('Sample')
        plt.ylabel('Force')
        plt.title('Force Data')
        plt.show()

if __name__ == "__main__":
    rospy.init_node('my_ft_sensor_test_node')
    my_ft_sensor = MyFTSensor()

    # Let the subscriber collect data for a certain time
    rospy.sleep(1)  # Adjust the time as needed

    while not rospy.is_shutdown():
        # print("force", my_ft_sensor.force)
        print(force2array(my_ft_sensor.modified_force_msg.wrench.force))
        rospy.sleep(0.5)