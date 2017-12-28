#!/usr/bin/env python
""" Random move strategy """

import random
import rospy

import move_helper
from move_strategy import MoveStrategy
from turtle_control import TurtleControl


class RandomMoveStrategy(MoveStrategy):
	""" Random move strategy based on random.random """

	def __init__(self):
		""" Ctor """
		MoveStrategy.__init__(self)


	def get_next(self):
		""" Move robot randomly """

		vel_msg = move_helper.get_zero_twist()

		# Decide if the turtle walks or turns
		turtle_walks = random.choice([True, False])

		# Velocity should be between -10 and 10, linear x (walk) or angular z (turn)
		veloc_value = random.choice(range(-10, 11))

		if turtle_walks:
			vel_msg.linear.x = veloc_value
		else:
			vel_msg.angular.z = veloc_value

		return vel_msg


if __name__ == "__main__":
	try:
		T_CONTROL = TurtleControl(RandomMoveStrategy, 2)
		rospy.loginfo("Starting random walker")
		T_CONTROL.run()
	except rospy.ROSInterruptException:
		pass
