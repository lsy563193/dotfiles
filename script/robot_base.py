#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import roslib
import sys
roslib.load_manifest('mybot')  
import rospy  
import robot_base_link

#import pdb
#_DEBUG=='True'
	
if __name__ == '__main__':
	try:
		robotbase=robot_base_link.Robot_Base()
		robotbase.robot_msg_pub()
	except rospy.ROSInterruptException: 
		rospy.loginfo("Robot Base ending")
