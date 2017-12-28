#!/usr/bin/env python
""" Random mover """

import random
import rospy

import move_helper
from turtle_interface import TurtleInterface

PI = 3.1415926535897


class RandomMover(object):
	""" Random mover based on random.random """

	turtle_if = None
	rate_limiter = None

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		self.turtle_if = TurtleInterface()
		self.rate_limiter = rospy.Rate(2)


	def move(self):
		""" Move robot randomly """

		rospy.loginfo("Starting random walker")

		# While loop to assure that Ctrl-C can exit the app
		while not rospy.is_shutdown():
			vel_msg = move_helper.get_zero_twist()

			# Decide if the turtle walks or turns
			turtle_walks = random.choice([True, False])

			# Velocity should be between -10 and 10, linear x (walk) or angular z (turn)
			veloc_value = random.choice(range(-10, 11))

			if turtle_walks:
				vel_msg.linear.x = veloc_value
			else:
				vel_msg.angular.z = veloc_value

			self.turtle_if.publish(vel_msg)
			self.rate_limiter.sleep()

		# Make sure to stop robot after the program has been cancelled
		self.turtle_if.publish(move_helper.get_zero_twist())


if __name__ == "__main__":
	try:
		T_MOVER = RandomMover()
		T_MOVER.move()
	except rospy.ROSInterruptException:
		pass
