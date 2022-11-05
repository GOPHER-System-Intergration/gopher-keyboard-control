#!/usr/bin/env python

from __future__ import print_function
from operator import and_

import rospy
import actionlib

from std_msgs.msg import String
from pynput import keyboard
from geometry_msgs.msg import Twist
from math import pi, sin, cos 

# Action Lib messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class MobileBase():
    def __init__(self):

        # self.twist_pub = rospy.Publisher('gopher/base_controller/command', Twist, queue_size=1)
        self.twist_pub = rospy.Publisher('base_controller/command', Twist, queue_size=1)

        self.current_lin_vel = 0.0
        self.current_rot_vel = 0.0

        self.target_lin_vel = 0.0
        self.target_rot_vel = 0.0

        self.current_lin_accel = 0.0
        self.current_rot_accel = 0.0
        
        self.current_lin_jerk = 0.0
        self.current_rot_jerk = 0.0

        self.max_lin_vel = 0.5 # meters
        self.min_lin_vel = -0.2 # meters
        self.max_rot_vel = pi/2.0 # rads per sec (90 degrees per sec)
        self.min_rot_vel = -pi/2.0

        self.max_lin_accel = self.calc_accel(vel = self.max_lin_vel, time = 1.0)
        self.min_lin_accel = -self.max_lin_accel  # this should be deceleration
        self.max_rot_accel = self.calc_accel(vel = self.max_rot_vel, time = 1.0)
        self.min_rot_accel = -self.max_rot_accel  # this should be deceleration

        self.smooth_out_lin_vel = 0.1
        self.smooth_out_rot_vel = pi/4.0

        self.pos_lin_jerk = self.calc_jerk( vel_change = self.smooth_out_lin_vel, 
                                            accel = self.max_lin_accel) 
        self.neg_lin_jerk = self.calc_jerk( vel_change = -self.smooth_out_lin_vel, 
                                            accel = self.min_lin_accel) 

        self.pos_rot_jerk = self.calc_jerk( vel_change = self.smooth_out_rot_vel, 
                                            accel = self.max_rot_accel) 
        self.neg_rot_jerk = self.calc_jerk( vel_change = -self.smooth_out_rot_vel, 
                                            accel = self.min_rot_accel) 
        
        
        self.previous_time = rospy.get_time()

    def zero_vel(self):
        self.current_lin_vel = 0.0
        self.current_rot_vel = 0.0

        self.target_lin_vel = 0.0
        self.target_rot_vel = 0.0

    def zero_accel(self):
        self.current_lin_accel = 0.0
        self.current_rot_accel = 0.0

    def calc_accel(self, vel, time):
        '''
        @params vel [m/sec or rad/sec]
        @params time [sec]

        returns - the acceleration
        '''
        return float(vel)/float(time)

    def calc_jerk(self, vel_change, accel):
        """
        calculates the needed jerk to make the targeted shift in the velocity:
        
        we dont really care how long it will take to reduce the speed to 0. 
        """
        # we will be doing a sodo intergral of the acceleration curve
        # vel_change = 0.5 * accel * time    --> time = vel_change / (0.5 * accel)
        # jerk = accel / time                --> time = accel / jerk
        #                                    --> vel_change / (0.5 * accel) = accel / jerk
        jerk = accel / vel_change * (0.5 * accel)
        return jerk

    def get_time_step_in_sec(self):

        current_time = rospy.get_time()
        duration = current_time - self.previous_time

        self.previous_time = current_time

        # print(str(duration))
        return duration


    def pub_vel(self, x_vel, w_vel):
        """
        Send the twist msg to the base
        """
        msg = Twist()
        msg.linear.x = x_vel
        msg.angular.z = w_vel

        self.current_lin_vel = x_vel
        self.current_rot_vel = w_vel
  
        self.twist_pub.publish(msg)

    def pub_target_vel(self):
        self.pub_vel(self.target_lin_vel, self.target_rot_vel)

    def update_target_vel(self):

        # lets consider the idea of changing the acceleration of the robot instead of worrying about other bs.
        time_step = self.get_time_step_in_sec()

        self.target_lin_vel = self.current_lin_vel + self.current_lin_accel*time_step
        self.target_rot_vel = self.current_rot_vel + self.current_rot_accel*time_step

        # I have always set the acceleration to be a non zero value, except for when I want the system to not move.
        # There are definietly better ways to do this but:

        if self.current_lin_accel == 0: # if we are not trying to accelerate
            self.target_lin_vel = 0
        else:
            # constraining the velocity of the mobile base
            if self.target_lin_vel > self.max_lin_vel: self.target_lin_vel = self.max_lin_vel
            elif self.target_lin_vel < self.min_lin_vel: self.target_lin_vel = self.min_lin_vel

        if self.current_rot_accel == 0: # if we are not trying to accelerate
            self.target_rot_vel = 0
        else:
            # constraining the velocity of the mobile base
            if self.target_rot_vel > self.max_rot_vel: self.target_rot_vel = self.max_rot_vel
            elif self.target_rot_vel < self.min_rot_vel: self.target_rot_vel = self.min_rot_vel

        # print(self.target_lin_vel, self.target_rot_vel)


    def hard_stop(self):
        self.pub_vel(0.0, 0.0)
        # Reseting all the current and target accelerations and velocities
        self.zero_accel()
        self.zero_vel()

    def keys_map_accel_control(self, keys):
        """
        Controls the velocity of the mobile base of the robot
        We are going to set the acceleration of the robot instead of the velocity instead
        """
        #                                           Linear Acceleration        Rotational Acceleration
        # Forward motion    (Keyboard keys : W               1                          ---
        # Backward motion   (Keyboard keys : S              -1                          ---
        # Turn Right        (Keyboard keys : D              ---                         -1
        # Turn Left         (Keyboard keys : A              ---                          1
        #              

        def choose_target_accel_toward_stopping(error_around_zero, smooth_out_vel_bound, target_vel, accel, deaccel, current_accel, pos_jerk, neg_jerk):
            # choosing the acceleration to slow down the base

            # i need some velocity to start smoothing at the vel curve
            

            # if error_around_zero > smooth_out_vel_bound:
            #     rospy.WARN("Error is too large. Consider lowering it below 0.1")

            # if target_vel > smooth_out_vel_bound: return deaccel
            # elif target_vel < smooth_out_vel_bound and target_vel > error_around_zero: return update_accel_using_jerk(current_accel, neg_jerk)
            # elif target_vel < -smooth_out_vel_bound: return accel
            # elif target_vel > -smooth_out_vel_bound and target_vel < - error_around_zero: return update_accel_using_jerk(current_accel, pos_jerk)
            # else: return 0.0

            if target_vel > error_around_zero: return deaccel
            # elif target_vel < smooth_out_vel_bound and target_vel > error_around_zero: return update_accel_using_jerk(current_accel, neg_jerk)
            elif target_vel < -error_around_zero: return accel
            # elif target_vel > -smooth_out_vel_bound and target_vel < - error_around_zero: return update_accel_using_jerk(current_accel, pos_jerk)
            else: return 0.0


        def update_accel_using_jerk(current_accel, jerk):
            time_step = self.get_time_step_in_sec()
            return current_accel + jerk*time_step

        def choose_lin_accel_toward_stopping():
            # we should choose a new acceleration in the attempot to slow down the base
            self.current_lin_accel = choose_target_accel_toward_stopping(   error_around_zero = 0.01,
                                                                            smooth_out_vel_bound = self.smooth_out_lin_vel, 
                                                                            target_vel = self.target_lin_vel,
                                                                            accel = self.max_lin_accel,
                                                                            deaccel = self.min_lin_accel,
                                                                            current_accel = self.current_lin_accel,
                                                                            pos_jerk = self.pos_lin_jerk,
                                                                            neg_jerk= self.neg_lin_jerk)
                                                            

        def choose_rot_accel_toward_stopping():
            # we should choose a new acceleration in the attempot to slow down the base
            self.current_rot_accel = choose_target_accel_toward_stopping(   error_around_zero = 0.05,
                                                                            smooth_out_vel_bound = self.smooth_out_rot_vel, 
                                                                            target_vel = self.target_rot_vel,
                                                                            accel = self.max_rot_accel,
                                                                            deaccel = self.min_rot_accel,
                                                                            current_accel = self.current_rot_accel,
                                                                            pos_jerk = self.pos_rot_jerk,
                                                                            neg_jerk= self.neg_rot_jerk)

        # forward / backward control

        if 'w' in keys and 's' in keys:
            choose_lin_accel_toward_stopping()
        elif 'w' in keys: 
            self.current_lin_accel = self.max_lin_accel
        elif 's' in keys: 
            self.current_lin_accel = self.min_lin_accel
        else:
            choose_lin_accel_toward_stopping()
        
        # rotation control

        if 'd' in keys and 'a' in keys:
            choose_rot_accel_toward_stopping()
        elif 'a' in keys: 
            self.current_rot_accel = self.max_rot_accel
        elif 'd' in keys: 
            self.current_rot_accel = self.min_rot_accel
        else:
            choose_rot_accel_toward_stopping()

        self.update_target_vel()
        self.pub_target_vel()


# ----------------------------------------------Action Client -------------------------------------------------------
# This code is base don the demo code provided by Fetch Robotics Github Repo: fetch_gazebo package

# Move base using navigation stack
class MoveBaseActionClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

        