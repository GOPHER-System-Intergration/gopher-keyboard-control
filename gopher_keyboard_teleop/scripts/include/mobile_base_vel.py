# The intent of this node is to publish the mobile base's current
#     lin vel and rotational velocity using the robot's joint states

#!/usr/bin/env python

from __future__ import print_function
from operator import and_

import rospy

from math import pi, sin, cos 

# describes the velocity, efforts of joints in the robot (a robot's wheels also being joints)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

wheel_radious = 0.10 # [meters] radious of the wheels
wheel_base = 0.15 # [meters] distance between the driving wheels

lin_vel_pub = rospy.Publisher("frieght_base/lin_vel", Float64, queue_size=1)
rot_vel_pub = rospy.Publisher("frieght_base/rot_vel", Float64, queue_size=1)

def calcVel(data = JointState):
    # TODO Assumed that forward rotation of each wheel is a posative value. Check on the robot if that is the case.

    # lin vel of the base the is average of the vel of each wheel
    average_rot_vel = (data.velocity[0] + data.velocity[1])/2.0
    lin_vel = average_rot_vel*wheel_radious
    
    rot_vel = (-1*data.velocity[0] + data.velocity[1])*wheel_radious/wheel_base

    lin_vel_pub.publish(Float64(lin_vel))
    rot_vel_pub.publish(Float64(rot_vel))

if "__name__" == "__main__":

    node = rospy.init_node("base_velocity_calc")

    rate = rospy.Rate(10)
    sub = rospy.Subscriber("jointStates", JointState, calcVel)


    # TODO Calc the lin vel of the base and publish
    # TODO Calc the rot vek of the base and publish

    # Keep the node alive for as long needed
    rospy.spin() 
    