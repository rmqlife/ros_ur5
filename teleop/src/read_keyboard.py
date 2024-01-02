#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class MyKeyboard:
    def __init__(self):
        self.pressed_keys = set()
        
        # Create a subscriber to receive keyboard input
        rospy.Subscriber('keyboard_input', String, self.keyboard_callback)

        self.volume = 0

    def keyboard_callback(self, data):
        key_str = data.data
        if key_str:
            self.pressed_keys.add(key_str)
            rospy.loginfo(f"Key pressed: {key_str}")

            if key_str == 'q':
                self.volume += 10
            
            if key_str == 'w':
                self.volume -= 10

            # Corrected the volume check
            if self.volume > 100:
                self.volume = 100
            elif self.volume < 0:
                self.volume = 0

if __name__ == '__main__':
    try:
        rospy.init_node('my_keyboard_subscriber', anonymous=True)
        my_keyboard = MyKeyboard()
        
        while not rospy.is_shutdown():
            # Corrected the volume variable reference
            print('volume is', my_keyboard.volume)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        pass
