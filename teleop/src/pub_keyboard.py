#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pynput import keyboard

class KeyboardInputPublisher:
    def __init__(self):
        rospy.init_node('keyboard_input_publisher', anonymous=True)
        
        # Create a publisher to publish keyboard input
        self.key_pub = rospy.Publisher('keyboard_input', String, queue_size=10)

        # Initialize the keyboard listener
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release
        )

    def on_key_press(self, key):
        try:
            # Publish the pressed key as a ROS message
            key_str = str(key.char)
            self.key_pub.publish(key_str)
            rospy.loginfo(f"Key pressed: {key_str}")
        except AttributeError:
            # Some keys may not have a char attribute (e.g., special keys)
            pass

    def on_key_release(self, key):
        # Handle key release if needed
        rospy.loginfo("Key released")

    def run(self):
        # Start the keyboard listener in a separate thread
        with self.keyboard_listener as listener:
            listener.join()

if __name__ == '__main__':
    try:
        node = KeyboardInputPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
