#!/usr/bin/env python

from __future__ import print_function
from operator import and_

import rospy
import actionlib
import tf



from std_msgs.msg import String
from pynput import keyboard
from geometry_msgs.msg import Twist
from math import pi, sin, cos 

# Action Lib messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


from include.mobile_base import MoveBaseActionClient

if __name__ == '__main__':
    
    
    rospy.init_node("move_base_action_client_node", anonymous=True)

    rate =  rospy.Rate(10) # 1hz
    client = MoveBaseActionClient()
    
    print("hello")

    while not rospy.is_shutdown():

        client.move_relative(1.0 ,pi/4.0)
        
        # print(tf_transformer.getFrameStrings())
        # print("hello")
        # try:

        #     print("hello")

        #     if tf_listener.frameExists('map'):
        #         rospy.loginfo("map exist")
        #     else:
        #         rospy.loginfo("map is not seen")


        #     if tf_listener.frameExists('/base_link'):
        #         rospy.loginfo("base_link works")
        #     else:
        #         rospy.loginfo("base_link is not seen")
            # relative_goal = PoseStamped()
            # relative_goal.header.stamp = rospy.Time.now()
            # relative_goal.header.frame_id = "base_link"
            
            # relative_goal.pose.position.x = x
            # relative_goal.pose.orientation.z = 1.0
            # relative_goal.pose.orientation.w = theta

            # move_goal = MoveBaseGoal()
            
            # # self.tf_listener.waitForTransform("/odom", "/base_link", rospy.Time.now(), rospy.Duration(10.0))
            # map_goal = self.transformer.transformPose(target_frame="map", ps=relative_goal)
            # move_goal.target_pose = map_goal

            # self.client.send_goal(move_goal)
            # self.client.wait_for_result()
            # rate.sleep()    

        # except:
        #     pass
    
        rate.sleep()
    
        
        