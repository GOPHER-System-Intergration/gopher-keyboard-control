#!/usr/bin/env python3

from __future__ import print_function

import rospy

import sys
from select import select

from std_msgs.msg import String
from pynput import keyboard
from geometry_msgs.msg import Twist
from math import pi
from include.mobile_base import MobileBase
from include.robot_arms import LeftArm, RightArm
from include.z_axis import ZAxis

class KeyboardControl():
    def __init__(self):

        rospy.init_node('keyboard_double3_publisher', anonymous=True)

        rospy.set_param("keyboard_status", {"isAlive":True, "isInit":True})

        rate = rospy.Rate(10) # 1hz

        self.mobile_base = MobileBase()
        self.left_arm = LeftArm()
        self.right_arm = RightArm()
        self.z_axis = ZAxis()
        

        # Control Flags

        self.left_arm_control_flag = False
        self.right_arm_control_flag = False
        self.base_control_flag = False
        self.z_axis_control_flag = False

        # Previous Key Pressed
        # self.previous_key = ''

        self.stop_robot()

        self.init_msg = """
        
        Freight Controller:

        Hold Correspondng Button to Keep the Robot Moving

        Press ESC to hard stop robot

        Press Cntrl-C to finish the programm

        """


    def keys_map_robot_control(self, keys):    
        """
        Maps the key inputs to parts of the robot for piece-wise control.
        The stop button (Shift) is used to stop the robot and sets all state flags off.
        The node will still run!
        """
        def keys_map(keys_left_arm, keys_right_arm, keys_mobile_base, keys_z_axis):
            self.left_arm.keys_map(keys_left_arm)
            self.right_arm.keys_map(keys_right_arm)
            self.mobile_base.keys_map_accel_control(keys_mobile_base)
            self.z_axis.keys_map(keys_z_axis)

        # def keys_map_state_update(self, keys):
        controller_keys = {"left", "right", "up", "down"}
        controller_keys_checked_and_pressed = []

        # counts the number of these keys is pressed
        for controller_key in controller_keys:
            if controller_key in keys:
                controller_keys_checked_and_pressed.append(controller_key)
        
        # if there is only on object being controlled, set that flag. Otherwise, change nothing
        if len(controller_keys_checked_and_pressed) == 1: self.key_map_state_update(controller_keys_checked_and_pressed[0])

        # self.keys_map_state_update(keys)

        if "esc" in keys:  # Our Emergency Stop Button
            self.reset_control_flags()
            self.stop_robot()

        elif self.left_arm_control_flag:  
            keys_map(keys, '', '', '')

        elif self.right_arm_control_flag: 
            keys_map('', keys, '', '')

        elif self.base_control_flag:      
            keys_map('', '', keys, '')

        elif self.z_axis_control_flag:    
            keys_map('', '', '', keys)

    

    def reset_control_flags(self):
        self.left_arm_control_flag = False
        self.right_arm_control_flag = False
        self.base_control_flag = False
        self.z_axis_control_flag = False

    def key_map_state_update(self, key):
        """
        Maps the key to the parts of the robot for the user to control:
            left -> left arm
            right -> right arm
            up -> z-axis
            down -> mobile base
        
        Note: 2 states CAN NOT be active at the same time.

        EX: Pressing LEFT and DOWN -> left arm
        """

        if key == "left":
            self.reset_control_flags()
            self.left_arm_control_flag = True
            print("IN_STATE_UPDATE -> left")
        elif key == "right":
            self.reset_control_flags()
            self.right_arm_control_flag = True
            print("IN_STATE_UPDATE -> right")
        elif key == "up":
            self.reset_control_flags()
            self.z_axis_control_flag = True
            print("IN_STATE_UPDATE -> z_axis")
        elif key == "down":
            self.reset_control_flags()
            self.base_control_flag = True
            print("IN_STATE_UPDATE -> base")

    def stop_robot(self):
        """
        Stop the robot as fast as possible
        """
        self.left_arm.pub_vel( 0.0,  0.0,  0.0,  0.0,  0.0,  0.0)
        self.right_arm.pub_vel( 0.0,  0.0,  0.0,  0.0,  0.0,  0.0)
        self.z_axis.pub_vel(0.0)
        self.mobile_base.hard_stop()

    def key_map_robot_control_on_release(self, key):
        def key_map_on_release(key_left_arm, key_right_arm, key_mobile_base, key_z_axis):
            self.left_arm.key_map_on_release(key_left_arm)
            self.right_arm.key_map_on_release(key_right_arm)
            self.mobile_base.key_map_on_release(key_mobile_base)
            self.z_axis.key_map_on_release(key_z_axis)

        try:

            if self.left_arm_control_flag:  
                key_map_on_release(key, '', '', '')

            elif self.right_arm_control_flag: 
                key_map_on_release('', key, '', '')

            elif self.base_control_flag:      
                key_map_on_release('', '', key, '')

            elif self.z_axis_control_flag:    
                key_map_on_release('', '', '', key)

        except Exception as e:
            rospy.logwarn(e)

    def keyboard_key_to_string(self,  key):
        try:
            if key == keyboard.Key.shift:
                # print('shift key!')
                return 'shift'
            elif key == keyboard.Key.esc:
                return 'esc'
            # elif key == keyboard.Key.tab:
            #     return "tab"
            elif key == keyboard.Key.down:
                # print('down key!')
                return 'down'
            elif key == keyboard.Key.up:
                # print('up key!')
                return 'up'
            elif key == keyboard.Key.right:
                # print('right key!')
                return 'right'
            elif key == keyboard.Key.left:
                # print('left key!')
                return 'left'


            elif key == keyboard.Key.f1:
                return "f1" 
            elif key == keyboard.Key.f2:
                return "f2" 
            elif key == keyboard.Key.f3:
                return "f3" 
            elif key == keyboard.Key.f4:
                return "f4" 
            elif key == keyboard.Key.f5:
                return "f5" 
            elif key == keyboard.Key.f6:
                return "f6" 
            else:
                return key.char.lower()
            # print('current key pressed', current_key)

        except AttributeError:
            # print('unregitered special key {0} pressed'.format(key))
            return ''


    def on_press(self, key):
        global current_keys
        global current_key
        # print(str(key))
        
        current_key = self.keyboard_key_to_string(key)
        current_keys.add(current_key)

    def on_release(self, key):
        global current_keys
        global current_key

        global released_key
        
        try:
            released_key = self.keyboard_key_to_string(key)
            current_keys.remove(released_key)

        except KeyError:
            pass
            current_key = ''
            released_key = ''
            # self.previous_key = ''


    def print_held_keys(self):
        print(current_keys)

    def on_shutdown(self):
        """
        This param will be shared with all the nodes in ros
        On the shutdown of this node, the system monitor will handle processinng and cleanly closing everything, 
        but it needs a signal of when to do so
        """
        
        # Notes : during this shutdown process there is no guarentee that publishing messages 
        #         or calling service will deliever msgs or request,
        #         thus it seems letting another node hadnle that process makes more sense

        #rospy.delete_param("keyboard_status")
        # alternative method:
        rospy.set_param("keyboard_status", {"isAlive":False, "isInit":True})

        # This param will be shared with all the nodes in ros
        # On the shutdown of this node, the system monitor will handle processinng and cleanly closing everything.

if __name__ == '__main__':


    current_key = ''
    current_keys = set()

    released_key = ''

    computer_keyboard = KeyboardControl()
    print(computer_keyboard.init_msg)

    # starts keyboard listener
    listener = keyboard.Listener(
        on_press=computer_keyboard.on_press,
        on_release=computer_keyboard.on_release)
    listener.start()

    try:
        while not rospy.is_shutdown():
            computer_keyboard.keys_map_robot_control(keys = current_keys)
            # computer_keyboard.print_held_keys()

            if not released_key == '':
                computer_keyboard.key_map_robot_control_on_release(released_key)
                released_key = ''
            


    except:
        pass

    rospy.on_shutdown(computer_keyboard.on_shutdown)
    listener.stop()