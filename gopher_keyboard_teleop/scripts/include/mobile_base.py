#!/usr/bin/env python

from __future__ import print_function
from operator import and_

import rospy
import actionlib
# import tf

import tf2_ros
import tf_conversions

from std_msgs.msg import String, Float32
from pynput import keyboard
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from math import pi, sin, cos 


# Action Lib messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class MobileBase():
    def __init__(self):

        # self.twist_pub = rospy.Publisher('gopher/base_controller/command', Twist, queue_size=1)
        self.twist_pub = rospy.Publisher('base_controller/command', Twist, queue_size=1)
        self.accel_pub = rospy.Publisher('base_controller/accel', Float32, queue_size=1)

        self.current_lin_vel = 0.0
        self.current_rot_vel = 0.0

        self.target_lin_vel = 0.0
        self.target_rot_vel = 0.0

        self.current_lin_accel = 0.0
        self.current_rot_accel = 0.0
        
        self.current_lin_jerk = 0.0
        self.current_rot_jerk = 0.0

        self.max_lin_vel = 0.25 # meters
        self.min_lin_vel = -0.1 # meters
        self.max_rot_vel = pi/2.0 # rads per sec (90 degrees per sec)
        self.min_rot_vel = -pi/2.0

        self.max_lin_accel = self.calc_accel(vel = self.max_lin_vel, time = 0.5)
        self.min_lin_accel = -self.max_lin_accel  # this should be deceleration
        self.max_rot_accel = self.calc_accel(vel = self.max_rot_vel, time = 0.5)
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

    def key_map_on_release(self, key):
        # TODO Added the response when a button is released
        pass
        # print(key)

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

            if target_vel > smooth_out_vel_bound: return deaccel
            elif target_vel > error_around_zero: return 0.5*deaccel
            elif target_vel < -smooth_out_vel_bound: return accel
            elif target_vel < -error_around_zero: return 0.5*accel
            else: return 0.0

        def choose_target_accel_toward_stopping_old_imp(error_around_zero, smooth_out_vel_bound, target_vel, accel, deaccel, current_accel, pos_jerk, neg_jerk):
            # choosing the acceleration to slow down the base

            # i need some velocity to start smoothing at the vel curve

            if target_vel > error_around_zero: return deaccel
           
            elif target_vel < -error_around_zero: return accel
            
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
            self.current_rot_accel = choose_target_accel_toward_stopping_old_imp(   error_around_zero = 0.05,
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
        
        try:
            accel = Float32()
            accel.data = self.current_lin_accel
            self.accel_pub.publish(accel)
        except:
            print("mess up")


        self.update_target_vel()
        self.pub_target_vel()


# ----------------------------------------------Action Client -------------------------------------------------------
# This code is base don the demo code provided by Fetch Robotics Github Repo: fetch_gazebo package

# Move base using navigation stack
class MoveBaseActionClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.goal_ros_pub=rospy.Publisher("debug/move_base/goal", TransformStamped, queue_size=1)

        rospy.loginfo("Waiting for move_base...")
        try:
            self.client.wait_for_server(rospy.Duration(2.0))
        except:
            rospy.logwarn("Took Too long for service to connect")
        rospy.loginfo("Server Connected")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer)

    def update_current_position(self, msg):

        self.current_est_position = msg

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

    def move_relative(self, x, theta):
        # TODO I should turn this into an action lib service to be easier for others to use
        # whats the goal relative to the base frame
        rospy.loginfo("Creating and populating transformation")

        t = TransformStamped()
        
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "relative_goal"
        t.transform.translation.x = x

        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, theta)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        rospy.loginfo("Sending Transformation")
            
        self.tf_broadcaster.sendTransform(t)

        rospy.loginfo("Sent")

        trans = TransformStamped()

        try:
            
            # move_goal = MoveBaseGoal()
            trans = self.buffer.lookup_transform("map", "relative_goal", rospy.Time(), rospy.Duration(2.0))
            
            # Debugging
            # self.goal_ros_pub.publish(trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)

        rospy.loginfo("Creating and populating MoveBaseGoal")

        move_goal = MoveBaseGoal()

        move_goal.target_pose.pose.position.x = trans.transform.translation.x
        move_goal.target_pose.pose.position.y = trans.transform.translation.y
        move_goal.target_pose.pose.position.z = trans.transform.translation.z

        
        move_goal.target_pose.pose.orientation.x = trans.transform.rotation.x
        move_goal.target_pose.pose.orientation.y = trans.transform.rotation.y
        move_goal.target_pose.pose.orientation.z = trans.transform.rotation.z
        move_goal.target_pose.pose.orientation.w = trans.transform.rotation.w
        move_goal.target_pose.header.frame_id = "map"
        move_goal.target_pose.header.stamp = rospy.Time.now()
            
        rospy.loginfo("Sending Goal")

        self.client.send_goal(move_goal)

        rospy.loginfo("Sent Goal, wating for result")
        
        self.client.wait_for_result()
    

        

    

        