#!/usr/bin/env python

from __future__ import print_function

import rospy

from std_msgs.msg import String
from pynput import keyboard
from geometry_msgs.msg import Twist
from math import pi

class ZAxis():
    def __init__(self):
        self.z_axis_twist_pub = rospy.Publisher('gopher/z_axis_controller/command', Twist, queue_size=1)

    def keys_map(self, keys):
        """
        Controls the velocity of the z-axis of the robot
        """

        # forward / backward control
        try:
            # TODO add relative functionality

            if "f1" in keys:
                pass
            if "f2" in keys:
                pass
            if "f3" in keys:
                pass
            if "f4" in keys:
                pass
            if "f5" in keys:
                pass
            if "f6" in keys:
                pass
            
            if 'w' in keys and 's' in keys: self.pub_vel(0.0)
            elif 'w' in keys:   self.pub_vel(1.0)
            elif 's' in keys:   self.pub_vel(-1.0)    
            else:               self.pub_vel( 0.0)
        except:
            self.pub_vel( 0.0)

    def pub_vel(self, z_vel):
        """
        Send the twist msg to the z-axis
        """
        msg = Twist()
        msg.linear.z = z_vel
        self.z_axis_twist_pub.publish(msg)

    def key_map_on_release(self, key):
        # TODO Added the response when a button is released
        print(key)


