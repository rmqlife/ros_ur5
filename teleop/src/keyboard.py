#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import threading
import sys, select, termios, tty

class MyKeyboard(threading.Thread):
    def __init__(self):
        super(MyKeyboard, self).__init__()
        self.shutdown_requested = False
        self.key_status = None  # To store the last key pressed

    # Function to read a single character from the terminal
    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        rospy.init_node('key_input_node')
        rate = rospy.Rate(10)  # Set the loop rate to 10 Hz

        print("Press 'Esc' key (ASCII code 27) to exit the program.")

        while not rospy.is_shutdown() and not self.shutdown_requested:
            c = self.getch()  # Read a single character from the terminal
            self.key_status = c  # Store the last key pressed
            rospy.loginfo(c)  # Log the character as a ROS info message
            rate.sleep()  # Sleep to control the loop rate

        print("Exiting the program...")

    def stop(self):
        self.shutdown_requested = True

if __name__ == '__main__':
    try:
        keyboard = MyKeyboard()
        keyboard.start()  # Start the keyboard thread

        while not rospy.is_shutdown():
            # Access the key status from the MyKeyboard instance
            key = keyboard.key_status
            if key and ord(key) == 27:  # Check if the character is 'Esc' (ASCII code 27)
                keyboard.stop()  # Request to stop the keyboard thread
                break  # Exit the loop if 'Esc' is pressed

    except rospy.ROSInterruptException:
        pass
