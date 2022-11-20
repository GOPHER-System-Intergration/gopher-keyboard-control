#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

class simpleService():
    def __init__(self):

        self.nh = rospy.init_node("simple_service")
        self.pub = rospy.Publisher("Publisher", Float64, queue_size=1)
        
        
        # keeps the node alive
        

    def handle_Empty(self, req):
        print("Request Recived")
        self.float = -1.0
        return EmptyResponse()

    def pub_float(self, float):
        self.pub.publish(float)

    

if __name__ == '__main__':
    
    node = simpleService()

    while not rospy.is_shutdown():

        if rospy.has_param("client_is_on"):
            status = rospy.get_param('client_is_on')["Operational"]
            if status:
                node.pub_float(1.0)
            else:


                node.pub_float(-1.0)
                rospy.delete_param("client_is_on")
                rospy.signal_shutdown("Client has shurdown")
                print("Shutting Down")


