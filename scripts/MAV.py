#!/usr/bin/env python
import rospy
import mavros
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist
from nav_msgs.msg import Odometry 
from std_msgs.msg import Empty
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode

from math import *
import numpy as np
from numpy.linalg import norm
import time

class MAV:
    def __init__(self):
        self.setpoint_velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=5)
        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=5)

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)

        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.setpoint_velocity = TwistStamped()
        self.setpoint_position = PoseStamped()
        self.current_pose = Pose()
        self.takeoff_height = 1.5
        self.land_height = 0.20
    

    def takeoff_position_hold(self):
        self.setpoint_velocity = TwistStamped()
        self.setpoint_velocity.linear.z = 0.5 (self.takeoff_height - self.current_pose.position.z)
        self.setpoint_velocity.stamp.header = rospy.Time.now()
        self.setpoint_velocity_pub.publish(self.setpoint_velocity)
        print("Setpoint Velcity: {}, {}, {}".format(self.setpoint_velocity.linear.x, 
                                                    self.setpoint_velocity.linear.y,
                                                    self.setpoint_velocity.linear.z ) )

    def land_position_hold(self):
        self.setpoint_velocity = TwistStamped()
        self.setpoint_velocity.linear.z = 0.5 (self.takeoff_height - self.current_pose.position.z)
        self.setpoint_velocity.stamp.header = rospy.Time.now()
        self.setpoint_velocity_pub.publish(self.setpoint_velocity)
        print("Setpoint Velcity: {}, {}, {}".format(self.setpoint_velocity.linear.x, 
                                                    self.setpoint_velocity.linear.y,
                                                    self.setpoint_velocity.linear.z ) )

    def drone_pose_callback(self, drone_pose_stamped):
        self.current_pose = drone_pose_stamped.pose
        print("Current Pose: {}, {}, {}".format(drone_pose_stamped.pose.position.x,
                                                drone_pose_stamped.pose.position.y, 
                                                drone_pose_stamped.pose.position.z ) )