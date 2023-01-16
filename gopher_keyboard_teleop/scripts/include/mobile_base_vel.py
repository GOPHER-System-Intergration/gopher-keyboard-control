# The intent of this node is to publish the mobile base's current
#     lin vel and rotational velocity using the robot's joint states

#!/usr/bin/env python

# Asummptions Checked
#   wheel joints
#       posative values for either joint result in forward movement of the robot
#       joint_state/velocity[0] -> left wheel
#       joint_state/velocity[1] -> right wheel

from __future__ import print_function
from operator import and_

import rospy

# describes the velocity, efforts of joints in the robot (a robot's wheels also being joints)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# This will check if we are intending to debug anything in the code. Mostly will be used for publishing individual wheel rot vel.
debugging = False
debugging = rospy.get_param("debug_freight_wheel_vel", True)

wheel_radious = 0.10 # [meters] radious of the wheels
wheel_base = 0.15 # [meters] distance between the driving wheels

lin_vel_pub = rospy.Publisher("frieght_base/lin_vel", Float64, queue_size=1)
rot_vel_pub = rospy.Publisher("frieght_base/rot_vel", Float64, queue_size=1)

debugger_left_wheel_pub = rospy.Publisher("freight_base/debugger/left", Float64, queue_size=1)
debugger_right_wheel_pub = rospy.Publisher("freight_base/debugger/left", Float64, queue_size=1)

joint_state = JointState()

def publishFreightCurretVel(data = JointState):
    # Saves the data of the joint state to the variable
    # only is used for debugging
    joint_state = data

    # lin vel of the base the is average of the vel of each wheel
    average_rot_vel = (data.velocity[0] + data.velocity[1])/2.0
    lin_vel = average_rot_vel*wheel_radious
    
    rot_vel = (-1*data.velocity[0] + data.velocity[1])*wheel_radious/wheel_base

    lin_vel_pub.publish(Float64(lin_vel))
    rot_vel_pub.publish(Float64(rot_vel))

if "__name__" == "__main__":

    node = rospy.init_node("base_velocity_calc")

    rate = rospy.Rate(10)
    sub = rospy.Subscriber("joint_states", JointState, publishFreightCurretVel)

    # should only really check once if we want to debug anything
    if debugging:
        while not rospy.is_shutdown():
            left_wheel_vel = joint_state.velocity[0] # rads/sec
            right_wheel_vel = joint_state.velocity[1] # rads/sec

            debugger_left_wheel_pub.publish(left_wheel_vel)
            debugger_right_wheel_pub.publish(right_wheel_vel)


    # Keep the node alive for as long needed
    rospy.spin() 
    