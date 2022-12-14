#!/usr/bin/env python

from __future__ import print_function

import rospy

from std_msgs.msg import String
from pynput import keyboard
from geometry_msgs.msg import Twist

    
class KeyboardControl():
    def __init__(self):

        self.twist_pub = rospy.Publisher('base_controller/command', Twist, queue_size=1)

        rospy.init_node('keyboard_double3_publisher', anonymous=True)
        
        rate = rospy.Rate(1) # 1hz

        self.init_msg = """
        
        Freight Controller:

        -------------------

        Moving Around:

                 W    
            A    S    D

        -------------------

        Hold Correspondng Button to Keep the Robot Moving

        Press ESC or Cntrl-C to finish the programm

        """

    def key_map(self, key):
        
        # Forward motion    (Keyboard keys : W and up-arrow)  -- {navigate, {throttle:'1', turn : '0'}}
        # Backward motion   (Keyboard keys : s and down-arrow)  -- {navigate, {throttle:'-1', turn : '0'}}
        # Turn Right        (Keyboard keys : D and right-arrow)  -- {navigate, {throttle:'0', turn : '0.5'}}
        # Turn Left         (Keyboard keys : A and left-arrow)  -- {navigate, {throttle:'0', turn : '-0.5'}}
        # 
        global msg
        msg = Twist()
                
        #print('key pressed : ', current_key)
        #if key == '':
        #    msg.linear.x = 0.0
        #    pub.publish(msg)
        #if key == 'space':
        #if key == '':
        #    msg.linear.x = 0.0
        #    pub.publish(msg) 
        if key == 'w':
            msg.linear.x = 1.0
            msg.angular.z = 0.0 
            self.twist_pub.publish(msg)
        if key == 's':
            msg.linear.x = -1.0
            msg.angular.z = 0.0
            self.twist_pub.publish(msg)
        if key == 'd':
            msg.linear.x = 0.0
            msg.angular.z = -1.0
            self.twist_pub.publish(msg)
        if key == 'a':
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            self.twist_pub.publish(msg)


    def on_press(self, key):
        global current_key
        try:
            if key == keyboard.Key.shift:
                print('shift key!')
                current_key = ''
            else:
                current_key = key.char
            #print('current key pressed', current_key)
        except AttributeError:
            print('key {0} pressed'.format(key))
            current_key = ''

    def on_release(self, key):
        global current_key
        current_key = ''
        if key == keyboard.Key.ctrl:
            return False
        if key == keyboard.Key.esc:
            return False
        else:
            current_key = ''

    

if __name__ == '__main__':
    current_key = ''
    computer_keyboard = KeyboardControl()
    print(computer_keyboard.init_msg)

    # starts keyboard listener
    listener = keyboard.Listener(
        on_press=computer_keyboard.on_press,
        on_release=computer_keyboard.on_release)
    listener.start()

    while listener.is_alive():
        print('key pressed : ', current_key)
        computer_keyboard.key_map(key = current_key)