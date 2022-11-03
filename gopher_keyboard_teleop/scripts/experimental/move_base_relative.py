#!/usr/bin/env python

from __future__ import print_function
from operator import and_

import rospy
import actionlib

from std_msgs.msg import String
from pynput import keyboard
from geometry_msgs.msg import Twist
from math import pi, sin, cos 

# Action Lib messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


from include.mobile_base import MoveBaseActionClient

if __name__ == '__main__':
    
    
    rospy.init_node("move_base_action_client_node", anonymous=True)
    client = MoveBaseActionClient()

    
    while not rospy.is_shutdown():
        client.goto(0.0, 0.0, pi/2.0)
        
        