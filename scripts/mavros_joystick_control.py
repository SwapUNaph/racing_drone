#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, Bool
from mavros_msgs.msg import RCIn
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist
import numpy as np
import time
from MAV import MAV

def takeoff_cb(empty):
    mav.setArm()
    mav.setMode('GUIDED')
    mav.setTakeoffMode()
    time.sleep(0.5)
    
def land_cb(empty):
    mav.setLandMode()
    time.sleep(0.5)
    
def emergency_cb(empty):
    land_cb(empty)
    mav.setDisarm()
    time.sleep(1)
    
def cmd_vel_cb(velocity):
    global autonomous_state
    if not autonomous_state:
	mav.set_velocity_twist(velocity)

    
def auto_cb(auto_active):
    global autonomous_state
    autonomous_state =  auto_active.data

if __name__ == '__main__':
    
    rospy.init_node("mavros_joystick_control", anonymous=False)
    mav = MAV()
    
    autonomous_state = False
    
    rospy.Subscriber("/bebop/takeoff", Empty, takeoff_cb)
    rospy.Subscriber("/bebop/land", Empty, land_cb)
    rospy.Subscriber("/bebop/cmd_vel", Twist, cmd_vel_cb)
    rospy.Subscriber("/bebop/emergency_shutdown", Empty, emergency_cb)
    rospy.Subscriber("/auto/autonomy_active", Bool, auto_cb)
    
    print("Mavros joystick control started.")
    
    while not rospy.is_shutdown():
	pass
