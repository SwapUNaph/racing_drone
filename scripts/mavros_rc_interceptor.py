#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, Bool
from mavros_msgs.msg import RCIn
import numpy as np
import time
from MAV import MAV


def rc_callback(rcin):
    global mav, system_state
    RCin = np.array(rcin.channels)
    
    if (RCin[7] < 1500): #H0
	system_state = "SEMI_AUTO"
    else:	#H1
	system_state = "MANUAL"
	
    if (RCin[8] < 1400) and (RCin[7] < 1500): #G2 and H0
	system_state = "SEMI_AUTO"
    elif (1400 < RCin[8] ) and (RCin[8] < 1600) and (RCin[7] < 1500): #G1 and H0
	system_state = "AUTO"
    else: #G0
	pass
	
	
	

if __name__ == "__main__":

    rospy.init_node('mavros_rc_interceptor', anonymous=True)
    mav = MAV()
    system_state = 'MANUAL'
    rospy.Subscriber("/mavros/rc/in", RCIn, rc_callback)
    enable_autonomoy_pub = rospy.Publisher("/auto/autonomy_active", Bool, queue_size=5)
    
    enable_auto = Bool()
    enable_auto.data = False

    while not rospy.is_shutdown():
	print("System state: %s" % system_state)
	if system_state == 'MANUAL':
	    pass
	elif system_state == 'SEMI_AUTO':
	    mav.setMode('GUIDED')
	    mav.set_velocity([0,0,0])
	    enable_auto = False
	else:
	    mav.setMode('GUIDED')
	    enable_auto = True
	    
	time.sleep(0.1)
	pass
