#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from MAV import MAV

def takeoff_callback(empty):
    global takeoff
    takeoff = True
    auto = False
    print("Takeoff hover...")

def land_callback(empty):
    global takeoff
    takeoff = False
    auto = False
    print("Land ...")

def auto_callback(empty):
    global auto
    auto = True
    print("Auto ...")

def manual_callback(empty):
    global auto
    auto = False
    takeoff = True
    print("Takeoff hover ...")

if __name__ == "__main__":

    rospy.init_node('bebop_mavros_bridge', anonymous=True)

    mav = MAV()

    takeoff = False

    rospy.Subscribe("/mav/takeoff", Empty, takeoff_callback)
    rospy.Subscribe("/mav/land", Empty, land_callback)
    rospy.Subscribe("/mav/auto", Empty, auto_callback)
    rospy.Subscribe("/mav/manual", Empty, manual_callback)

    while not rospy.is_shutdown():
        if takeoff and not auto:
            mav.takeoff_position_hold()
        else
            mav.land_position_hold()