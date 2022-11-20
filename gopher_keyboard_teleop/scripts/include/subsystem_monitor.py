#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class SubsystemMonitor():
    
    def __init__(self):
        
        print("Inicializing Subsystem Monitor")

        self.nh = rospy.init_node("gopher_subsystem_monitor")

        self.rate = rospy.Rate(10)

        self.base_status_name = "base_status"
        self.left_arm_status_name = "left_arm_status"
        self.right_arm_status_name = "right_arm_status"
        self.z_axis_status_name = "z_axis_status"
    
        # check the existance of param of a component
        self.init_robot_system_params()


    def init_robot_system_params(self):
        # iniciallizes all rosparams used to monitor the status of easch node
        #   if the ros param does not exist create rosparm: {isAlive: False, isInit: false}
        def create_param_if_needed(param):
            if not rospy.has_param(param):
                rospy.logwarn("%s does not exist yet, creating an empty status", param)
                rospy.set_param(param, {"isAlive":False, "isInit":False})

        create_param_if_needed(self.base_status_name)
        create_param_if_needed(self.left_arm_status_name)
        create_param_if_needed(self.right_arm_status_name)
        create_param_if_needed(self.z_axis_status_name)


    def handle_notAlive_status_events(self):
        # checks for the status of params and sends the needed shutdown request as needed

        def not_okay(param):
            status  = rospy.get_param(param)
            if not status["isAlive"] and status["isInit"]:
                return True

        try:        

            if not_okay(self.base_status_name) and rospy.has_param(self.base_status_name):
                self.shutdown_base()

            if not_okay(self.left_arm_status_name) and rospy.has_param(self.left_arm_status_name):
                self.shutdown_left_arm()

            if not_okay(self.right_arm_status_name) and rospy.has_param(self.right_arm_status_name):
                self.shutdown_right_arm()

            if not_okay(self.z_axis_status_name) and rospy.has_param(self.z_axis_status_name):
                self.shutdown_z_axis()
        except:
            pass
            

    def shutdown_base(self):
        rospy.delete_param(self.base_status_name)
        rospy.loginfo("Shutting Down %s", self.base_status_name)

    def shutdown_left_arm(self):
        rospy.delete_param(self.left_arm_status_name)
        rospy.loginfo("Shutting Down %s", self.left_arm_status_name)

    def shutdown_right_arm(self):
        rospy.delete_param(self.right_arm_status_name)
        rospy.loginfo("Shutting Down %s", self.right_arm_status_name)

    def shutdown_z_axis(self):
        rospy.delete_param(self.z_axis_status_name)
        rospy.loginfo("Shutting Down %s", self.z_axis_status_name)

    def shutdown(self):

        def delete_param(param):
            try:
                rospy.delete_param(param)
            except:
                print("error on %s", param)
                pass

        rospy.loginfo("Deleting Params")

        delete_param(self.base_status_name)
        delete_param(self.left_arm_status_name)
        delete_param(self.right_arm_status_name)
        delete_param(self.z_axis_status_name)

    def __del__(self):
        print("Deleting Subsystem Monitor")


if __name__ == "__main__":
    

    monitor = SubsystemMonitor()

    while not rospy.is_shutdown():
        monitor.handle_notAlive_status_events()
        monitor.rate.sleep()

    rospy.on_shutdown(monitor.shutdown)

    del monitor