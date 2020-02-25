#!/usr/bin/env python
import rospy
import mavros
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist
from nav_msgs.msg import Odometry 
from std_msgs.msg import Empty
from mavros_msgs.msg import State 
import mavros_msgs
from mavros_msgs.srv import CommandBool, SetMode

from math import *
import numpy as np
from numpy.linalg import norm
import time



class MAV:
    def __init__(self):
        self.setpoint_velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=5)
        self.setpoint_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=5)
        self.setpoint_attitude_pub = rospy.Publisher('/mavros/setpoint_attitude', PoseStamped, queue_size=5)

        # rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
	rospy.Subscriber('/mavros/state', State, self.state_callback)

        self.setpoint_velocity = TwistStamped()
        self.setpoint_position = PoseStamped()
        self.current_pose = Pose()
	
	self.current_state = State()
	self.takeoff_state = False
	self.auto_state = False
    
    def set_position(self, position):
	self.setpoint_position = TwistStamped()
	self.setpoint_position.pose.position.x = position[0]
	self.setpoint_position.pose.position.y = position[1]
	self.setpoint_position.pose.position.z = position[2]
	self.setpoint_position.header.stamp = rospy.Time.now()
	self.setpoint_position.publish(self.setpoint_position)
	print("Setpoint Position Sent: {}, {}, {}".format(position[0], position[1], position[2]) )
	
    def set_velocity(self, velocity):
	self.setpoint_velocity = TwistStamped()
	self.setpoint_velocity.twist.linear.x = velocity[0]
	self.setpoint_velocity.twist.linear.y = velocity[1]
	self.setpoint_velocity.twist.linear.z = velocity[2]
	self.setpoint_velocity.header.stamp = rospy.Time.now()
	self.setpoint_velocity_pub.publish(self.setpoint_velocity)
   
	print("Setpoint Velocity Sent: {}, {}, {}".format(self.setpoint_velocity.twist.linear.x, 
						    self.setpoint_velocity.twist.linear.y,
						    self.setpoint_velocity.twist.linear.z ) )
	
    def set_velocity_twist(self, velocity):
	self.setpoint_velocity = TwistStamped()
	self.setpoint_velocity.twist = velocity
	self.setpoint_velocity.header.stamp = rospy.Time.now()
	self.setpoint_velocity_pub.publish(self.setpoint_velocity)
   
	print("Setpoint Velocity Sent: {}, {}, {}".format(self.setpoint_velocity.twist.linear.x, 
						    self.setpoint_velocity.twist.linear.y,
						    self.setpoint_velocity.twist.linear.z ) )
	
    def setMode(self, mode):
	if self.current_state.mode != mode:
	    rospy.wait_for_service('/mavros/set_mode')
	    try:
	       flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
	       isModeChanged = flightModeService(custom_mode=mode) #return true or false
	    except rospy.ServiceException, e:
	       print "service set_mode call failed: %s. %s Mode could not be set. Check that GPS is enabled"% (e, mode)

    def setArm(self):
       rospy.wait_for_service('/mavros/cmd/arming')
       try:
	   armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
	   armService(True)
       except rospy.ServiceException, e:
	   print "Service arm call failed: %s"%e

    def setDisarm(self):
       rospy.wait_for_service('/mavros/cmd/arming')
       try:
	   armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
	   armService(False)
       except rospy.ServiceException, e:
	   print "Service arm call failed: %s"%e
	   
	   
    def drone_pose_callback(self, drone_pose_stamped):
	self.current_pose = drone_pose_stamped.pose
	

    def setTakeoffMode(self):
       rospy.wait_for_service('/mavros/cmd/takeoff')
       try:
	   takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
	   takeoffService(altitude = 1.5, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
       except rospy.ServiceException, e:
	   print "Service takeoff call failed: %s"%e
	   
    def setLandMode(self):
       rospy.wait_for_service('/mavros/cmd/land')
       try:
	   landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
	   #http://wiki.ros.org/mavros/CustomModes for custom modes
	   isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
       except rospy.ServiceException, e:
	   print "service land call failed: %s. The vehicle cannot land "%e

    def state_callback(self, mav_state):
	self.current_state = mav_state
	
