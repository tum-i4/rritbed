#!/usr/bin/env python
""" Interface for publishing to turtle """

import rospy
from geometry_msgs.msg import Twist

class TurtleInterface(object):
	""" Direct interface class for moving a turtle """

	turtle_path = ""
	velocity_publisher = None


	def __init__(self, turtle_path="turtle1"):
		""" Ctor """

		object.__init__(self)

		self.turtle_path = turtle_path

		# Start our movement node
		rospy.init_node("robot_mover", anonymous=True)
		self.velocity_publisher = rospy.Publisher(turtle_path + "/cmd_vel", Twist, queue_size=10)


	def publish(self, vel_msg):
		""" Publish velocity to turtle """
		self.velocity_publisher.publish(vel_msg)
