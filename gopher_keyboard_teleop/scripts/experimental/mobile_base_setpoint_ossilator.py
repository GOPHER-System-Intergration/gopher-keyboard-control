#!/usr/bin/env python

from __future__ import print_function
from operator import and_

import rospy

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Imu



if __name__ == '__main__':
    
    
    rospy.init_node("mobile_base_setpoint_oscillator")
    pub = rospy.Publisher("/rot_controller/setpoint", Float64, queue_size=1)
    

    rate =  rospy.Rate(10) # 1hz
    state = 0

    lin_vel = 0.0
    speed = 0.1

    accel = 0.0
    a = 0.5 # meters/sec sq
    
    print("hello")

    try:
        # rospy.wait_for_message("/imu", Imu, timeout=5)

        # step input

        while not rospy.is_shutdown(): 
            pass
            
            if state == 0:
                lin_vel = speed
            elif state == 1:
                lin_vel = 0.0
            elif state == 2:
                lin_vel = speed * -1
            elif state == 3:
                lin_vel = 0.0
            elif state == 4:
                lin_vel = speed
                state = 0

            else:
                rospy.is_shutdown

            for i in range(100):
                pub.publish(Float64(lin_vel))
                rospy.sleep(3.0 / 100.0)
            
            state += 1

        # ramp input 
        # I wanted to see if the ramp would reduce the shaking of the robot at the start of motion

        # TODO Check what happens if accel is increased or descreased in the 

        # while not rospy.is_shutdown(): 
            
            
        #     if state == 0:
        #         accel = 0.25
        #     elif state == 1:
        #         accel = 0
        #     elif state == 2:
        #         accel = -0.25
        #     elif state == 3:
        #         accel = 0.0
        #     elif state == 4:
        #         accel = 0.25
        #         state = 0

        #     else:
        #         rospy.is_shutdown

        #     for i in range(100):
        #         duration = 3.0 / 100.0
        #         lin_vel += accel*(duration)
        #         pub.publish(Float64(lin_vel))
        #         rospy.sleep(duration)
            
        #     state += 1

            
    except rospy.ROSException:
        rospy.logwarn("timeout occured")

        
        
        
    
        
        