#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from myOmni import MyBag

class MyRealSense(MyBag):
    def __init__(self, topic='/camera/color/image_raw', filename="realsense"):
        # Call the constructor of the parent class (MyBag)
        super().__init__(topic, filename)

        # Initialize the ROS node and OpenCV window
        self.bridge = CvBridge()
        # Create a subscriber to capture images from the RealSense camera
        rospy.Subscriber(topic, Image, self.image_callback)
        rospy.sleep(1)
        # cv2.namedWindow("RealSense Image", cv2.WINDOW_NORMAL)

    def display(self):
        try:
            # Display the OpenCV image using OpenCV
            cv2.imshow("RealSense Image", self.cv_image)
            cv2.waitKey(1)  # Adjust the delay as needed
        except Exception as e:
            rospy.logerr(e)

    def image_callback(self, data):
        super().subscriber_callback(data)
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        rospy.init_node('realsense_image_viewer')
        myRS = MyRealSense('/camera/color/image_raw', 'realsense_data')
        myRS.start_bag_recording()
        while not rospy.is_shutdown():
            myRS.display()
            rospy.sleep(0.05)
        myRS.stop_bag_recording()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()  # Close the OpenCV window when the program exits
