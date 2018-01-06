#!/usr/bin/env python
"""
Random move strategy
Possible arguments:
-pi     : Use 3.14... as seed
-pi1000 : use 314... as seed
[seed]  : use given value (must be valid float or int) as seed
"""

import sys
import random
import rospy

import move_helper
from move_strategy import MoveStrategy
from turtle_control import TurtleControl


class RandomMoveStrategy(MoveStrategy):
	""" Random move strategy based on random.random """

	_rand_gen = random.Random()


	def __init__(self):
		""" Ctor """

		MoveStrategy.__init__(self)

		# Remove remapping arguments and program name
		args = rospy.myargv(sys.argv)[1:]

		seed = self._get_seed(args)

		if seed is not None:
			rospy.loginfo("Using seed %s", seed)
			self._rand_gen.seed(seed)


	def _get_seed(self, args):
		""" Get the seed from the supplied arguments (return None if no arguments are given) """

		if not args:
			return None

		if args[0] == "-pi":
			return 3.1415926535897

		if args[0] == "-pi1000":
			return 31415926535897

		try:
			seed = float(args[0])
		except ValueError:
			raise Exception(
				"Please provide valid argument or a float as a seed input.\nProvided: %s", args[0])

		return seed


	def get_next(self):
		""" Move robot randomly """

		vel_msg = move_helper.get_zero_twist()

		# Decide if the turtle walks or turns
		turtle_walks = self._rand_gen.choice([True, False])

		# Velocity should be between -10 and 10, linear x (walk) or angular z (turn)
		veloc_value = self._rand_gen.choice(range(-10, 11))

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
