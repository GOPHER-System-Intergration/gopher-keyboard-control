#!/usr/bin/env python

from __future__ import print_function
from operator import and_

import rospy

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Imu

def callback(data):
    print("data reviced")

if __name__ == '__main__':
    
    
    rospy.init_node("mobile_base_setpoint_oscillator")
    pub = rospy.Publisher("/lin_controller/setpoint", Float64, queue_size=1)
    sub = rospy.Subscriber("/imu", Imu, callback)

    rate =  rospy.Rate(10) # 1hz
    
    print("hello")

    try:
        rospy.wait_for_message("/imu", Imu, timeout=5)
        while not rospy.is_shutdown(): 
            pass

            # pub.publish(Float64(0.25))
            # rospy.sleep(4.0)
            # pub.publish(Float64(-0.25))
            # rospy.sleep(4.0)
            # pub.publish(Float64(0.0))
            # rospy.sleep(4.0)
    except rospy.ROSException:
        rospy.logwarn("timeout occured")

        
        
        
    
        
        