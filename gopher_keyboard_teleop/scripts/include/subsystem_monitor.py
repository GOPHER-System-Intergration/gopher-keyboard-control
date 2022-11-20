#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class SubsystemMonitor():

    base_status_name = "base_status"
    left_arm_status_name = "left_arm_status"
    right_arm_status_name = "right_arm_status"
    z_axis_status_name = "z_axis_status"
    
    def __init__(self):
        
        print("Inicializing Subsystem Monitor")

        self.nh = rospy.init_node("gopher_subsystem_monitor")

        self.rate = rospy.Rate(10)

        # check the existance of param of a component
        self.init_robot_system_params()
    
    def create_param_if_needed(self, param, isAlive=False, isInit=False):
        if not rospy.has_param(param):
            rospy.logwarn("%s does not exist yet, creating an empty status", param)
            rospy.set_param(param, {"isAlive":isAlive, "isInit":isInit})


    def init_robot_system_params(self):
        # iniciallizes all rosparams used to monitor the status of easch node
        #   if the ros param does not exist create rosparm: {isAlive: False, isInit: false}
        
        self.create_param_if_needed(self.base_status_name)
        self.create_param_if_needed(self.left_arm_status_name)
        self.create_param_if_needed(self.right_arm_status_name)
        self.create_param_if_needed(self.z_axis_status_name)

    def status_not_okay(self, param):
        try:
            status  = rospy.get_param(param)
            return not status["isAlive"] and status["isInit"]
        except:
            # if we can't get the parameter, then the parameter most likely does not exist:
            # thus the status of the param is not okay
            return True

    def handle_robot_status_events(self):
        # checks for the status of params and sends the needed shutdown request as needed
        try:
            for sub in [self.base_status_name, 
                        self.left_arm_status_name, 
                        self.right_arm_status_name, 
                        self.z_axis_status_name]:
                
                if self.status_not_okay(sub):
                    
                #     # Either a node has indicated (with ros param) they have shutdown
                #     # Any sub system shutdown will cause the robot to fail
                #     # Best to do all the needed actions to kill the robot before this node is shut down as well

                    self.shutdown_robot()
        except:
            pass
            

    def shutdown_robot(self):

        # Calls all the neeed services and topics to kill the robot
        print("Killing Robot")
        self.shutdown_base()
        self.shutdown_left_arm()
        self.shutdown_right_arm()
        self.shutdown_z_axis()

        # Kills the node
        print("Signalling Shutdown")
        rospy.signal_shutdown("death")
        

    def shutdown_base(self):
        try:
            pass
            # TODO Publish needed commands to systems to shut down the robot
        except:
            pass

    def shutdown_left_arm(self):
        try:
            pass
            # TODO Publish needed commands to systems to shut down the robot
        except:
            pass

    def shutdown_right_arm(self):
        try:
            pass
            # TODO Publish needed commands to systems to shut down the robot
        except:
            pass

    def shutdown_z_axis(self):
        try:
            pass
            # TODO Publish needed commands to systems to shut down the robot
        except:
            pass

    def del_params(self):

        def delete_param(param):
            try:
                rospy.delete_param(param)
            except:
                pass

        rospy.loginfo("Deleting Params")

        delete_param(self.base_status_name)
        delete_param(self.left_arm_status_name)
        delete_param(self.right_arm_status_name)
        delete_param(self.z_axis_status_name)


    # def __del__(self):
    #     print("Deleting Subsystem Monitor")


if __name__ == "__main__":
    

    monitor = SubsystemMonitor()

    while not rospy.is_shutdown():
        monitor.handle_robot_status_events()
        monitor.rate.sleep()

    rospy.on_shutdown(monitor.del_params)

    del monitor