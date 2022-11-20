#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class simpleClient():
    def __init__(self):

        self.nh = rospy.init_node("simple_client")
        rospy.set_param("client_status", {'Operational':True, "Dic2":"Nope"})

    def handle_shutdown(self):

        print("Start shutdown process")
        rospy.set_param("client_is_on", {'Operational':False, "Dic2":"Nope"})
    

if __name__ == '__main__':
    
    node = simpleClient()
    rospy.spin()

    rospy.on_shutdown(node.handle_shutdown)

    
