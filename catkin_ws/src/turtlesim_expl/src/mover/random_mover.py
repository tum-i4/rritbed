#!/usr/bin/env python
"""
Random move strategy
Possible arguments:
-pi     : Use 3.14... as seed
-pi1000 : use 314... as seed
[seed]  : use given value (must be valid float or int) as seed
"""

import argparse
import sys
import random
import rospy

import move_helper
from move_strategy import MoveStrategy
from turtle_control import TurtleControl
from turtlesim.msg import Color, Pose

POSE_PATH = "turtle1/pose"


class RandomMoveStrategy(MoveStrategy):
	""" Random move strategy based on random.random """

	_rand_gen = random.Random()

	_last_pose_field = "last_pose"
	_last_colour_field = "last_colour"
	_turtle_state = {
		_last_pose_field: None,
		_last_colour_field: None
	}


	def __init__(self):
		""" Ctor """

		MoveStrategy.__init__(self)

		# Remove remapping arguments and program name
		filtered_argv = rospy.myargv(sys.argv)[1:]

		parser = argparse.ArgumentParser(description="Randomly move a turtlesim around")

		group = parser.add_mutually_exclusive_group()
		group.add_argument("--seed", "-s", metavar="seed", type=float,
							help="Specify seed for the random generator")
		group.add_argument("-pi", action="store_const", dest="seed", const=3.1415926535897,
							help="Use pi as seed")
		group.add_argument("-pi1000", action="store_const", dest="seed", const=31415926535897.0,
							help="Use pi*10B as seed")

		args = parser.parse_args(filtered_argv)

		if args.seed is not None:
			rospy.loginfo("Using seed %s", args.seed)
			self._rand_gen.seed(args.seed)
		else:
			rospy.loginfo("No seed specified")

		rospy.Subscriber(POSE_PATH, Pose, self._save_pose)


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


	def _save_pose(self, pose):
		print(pose)
		self._set_last_pose(pose)


	# Getters & setters


	def _get_last_colour(self):
		return self._turtle_state[self._last_colour_field]


	def _set_last_colour(self, colour):
		assert(issubclass(colour, Color))
		self._turtle_state[self._last_colour_field] = colour


	def _get_last_pose(self):
		return self._turtle_state[self._last_pose_field]


	def _set_last_pose(self, pose):
		assert(issubclass(pose, Pose))
		self._turtle_state[self._last_pose_field] = pose


if __name__ == "__main__":
	try:
		T_CONTROL = TurtleControl(RandomMoveStrategy, 2)
		rospy.loginfo("Starting random walker")
		T_CONTROL.run()
	except rospy.ROSInterruptException:
		pass
