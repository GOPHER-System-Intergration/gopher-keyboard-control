#!/usr/bin/env python

from __future__ import print_function

import rospy

from std_msgs.msg import String
from pynput import keyboard
from geometry_msgs.msg import Twist
from kortex_driver.srv import *
from kortex_driver.msg import *
from math import pi

# Based on Kinevo Robot Arms which use the following documentation:
# https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/readme.md

# --------------------------------------------- Robot Arm -------------------------------

class RobotArm():
    def __init__(self, pub_topic_name, data_class):
        self.twist_command_pub = rospy.Publisher(pub_topic_name, data_class, queue_size=1)
    
    def keys_map(self, keys):
        """
        Controls the velocity of the end-effector of the corresponding arm

        @param isLeftArm: [bool] tells if the arm that is being controled is the left arm
            true -> left arm
            false -> right arm
        """
        # if(isLeftArm): print('left_arm_control-key pressed : ', current_key)
        # else: print('right_arm_control-key pressed : ', current_key)

        def check_key_pairs_for_vel(key1, key2):
            if key1 in keys and key2 in keys: return 0.0
            elif key1 in keys: return 1.0
            elif key2 in keys: return -1.0
            else: return 0.0

        # position control
        try:
            x_vel = check_key_pairs_for_vel('w', 's') 
            y_vel = check_key_pairs_for_vel('d', 'a') 
            z_vel = check_key_pairs_for_vel('e', 'q') 
            roll_vel  = check_key_pairs_for_vel('l', 'j') 
            pitch_vel = check_key_pairs_for_vel('i', 'k')
            yaw_vel   = check_key_pairs_for_vel('o', 'u')
            self.pub_vel(x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel)
        except:
            self.pub_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def pub_vel(self, x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel):
        """
        Send the twist msg to the arm
        """
        msg = TwistCommand()

        twist_msg = Twist()

        twist_msg.linear_x = x_vel
        twist_msg.linear_y = y_vel
        twist_msg.linear_z = z_vel
        
        twist_msg.angular_x = roll_vel
        twist_msg.angular_y = pitch_vel
        twist_msg.angular_z = yaw_vel

        msg.reference_frame = 0
        msg.twist = twist_msg
        msg.duration = 0

        self.twist_command_pub.publish(msg)

# --------------------------------------------- Left or Right Arm (Inherits Robot Arm's Methods)--------------------------------
class LeftArm(RobotArm):
    def __init__(self):
        self.twist_command_pub = rospy.Publisher('/my_gen3_left_arm/in/cartesian_velocity', TwistCommand, queue_size=1)

class RightArm(RobotArm):
    def __init__(self):
        self.twist_command_pub = rospy.Publisher('/my_gen3_right_arm/in/cartesian_velocity', TwistCommand, queue_size=1)
        

