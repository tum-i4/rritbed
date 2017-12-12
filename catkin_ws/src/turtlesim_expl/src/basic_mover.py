#!/usr/bin/env python
""" Basic mover """

import os
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def move():
	# Start a new node
	rospy.init_node('robot_mover', anonymous=True)
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()
	
	pass

if __name__ == '__main__':
	try:
		move()
	except rospy.ROSInterruptException:
		pass
