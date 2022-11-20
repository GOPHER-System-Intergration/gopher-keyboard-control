#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from include.subsystem_monitor import SubsystemMonitor

class KeyboardSubsytemMonitor(SubsystemMonitor):

    keyboard_state_name = "keyboard_status"

    def __init__(self):

        print("Inicializing Keyboard-Subsystem Monitor")

        self.nh = rospy.init_node("gopher_keyboard_subsystem_monitor")

        self.rate = rospy.Rate(10)

        # check the existance of param of a component
        self.init_robot_system_params()

        # init keyboard params
        self.create_param_if_needed(self.keyboard_state_name)

    def handle_keyboard_status_events(self):

        if self.status_not_okay(self.keyboard_state_name):
            self.shutdown_robot()

    def del_monitor_params(self):
        # deletes all the paramaters

        try:
            rospy.delete_param(self.keyboard_state_name)
        except:
            print("error on %s", self.keyboard_state_name)
        
        self.del_params()


    # def __del__(self):
    #     print("Deleting Keyboard-Subsystem Monitor")


if __name__ == "__main__":

    monitor = KeyboardSubsytemMonitor()

    while not rospy.is_shutdown():
        monitor.handle_keyboard_status_events()
        monitor.handle_robot_status_events()
        monitor.rate.sleep()

    rospy.on_shutdown(monitor.del_monitor_params)

    del monitor


    