#!/usr/bin/env python

# The intent of this node is to publish the mobile base's current
#     lin vel and rotational velocity using the robot's joint states


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

# debugging = rospy.get_param("debug_freight_wheel_vel", True)

class talker():
    
    wheel_radious = 0.055 # [meters] radious of the wheels
    wheel_base = 0.355 # [meters] distance between the driving wheels
    # checked Jan 16 2023

    def __init__(self):
        
        
        node = rospy.init_node("base_velocity_calc")
        rate = rospy.Rate(10)
        
        self.lin_vel_pub = rospy.Publisher("/lin_controller/state", Float64, queue_size=1)
        self.rot_vel_pub = rospy.Publisher("/rot_controller/state", Float64, queue_size=1)
        rospy.Subscriber("joint_states", JointState, self.publishFreightCurretVel)
        rospy.spin() 

    def publishFreightCurretVel(self, data = JointState):
        # lin vel of the base the is average of the vel of each wheel
        average_rot_vel = (data.velocity[0] + data.velocity[1])/2.0
        lin_vel = average_rot_vel*self.wheel_radious
        
        rot_vel = (-1*data.velocity[0] + data.velocity[1])*self.wheel_radious/self.wheel_base

        self.lin_vel_pub.publish(Float64(lin_vel))
        self.rot_vel_pub.publish(Float64(rot_vel))

    

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass