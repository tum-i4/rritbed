#!/usr/bin/env python
"""
Random move strategy
Possible arguments:
-pi     : Use 3.14... as seed
-pi1000 : use 314... as seed
[seed]  : use given value (must be valid float or int) as seed
"""

import argparse
import random
import sys
import time

import rospy

import move_helper
from move_strategy import MoveStrategy
from turtle_control import TurtleControl
from turtlesim.msg import Color, Pose

POSE_PATH = "turtle1/pose"
COLOUR_PATH = "turtle1/color_sensor"


class RandomMoveStrategy(MoveStrategy):
	""" Random move strategy based on random.random """

	_rand_gen = random.Random()

	_last_pose_field = "last_pose"
	_last_colour_field = "last_colour"
	_data_field = "data"
	_last_update_field = "last_update"

	_update_rate_in_sec = 0.01

	_turtle_state = {
		_last_pose_field: {
			_data_field: None,
			_last_update_field: 0
		},
		_last_colour_field: {
			_data_field: None,
			_last_update_field: 0
		}
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

		# TODO: Add argument for intelligence mode

		args = parser.parse_args(filtered_argv)

		if args.seed is not None:
			rospy.loginfo("Using seed %s", args.seed)
			self._rand_gen.seed(args.seed)
		else:
			rospy.loginfo("No seed specified")

		rospy.Subscriber(POSE_PATH, Pose, self._save_pose)
		rospy.Subscriber(COLOUR_PATH, Color, self._save_colour)


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
		self._set_last_pose(pose)


	def _save_colour(self, colour):
		self._set_last_colour(colour)


	# Getters & setters


	def _get_last_pose(self):
		return self._turtle_state[self._last_pose_field]


	def _set_last_pose(self, pose):
		self._set_field(self._last_pose_field, pose, Pose)


	def _get_last_colour(self):
		return self._turtle_state[self._last_colour_field]


	def _set_last_colour(self, colour):
		self._set_field(self._last_colour_field, colour, Color)


	def _set_field(self, field, data, data_class):
		assert(issubclass(data.__class__, data_class))

		time_now = time.clock()
		if time_now < self._update_rate_in_sec + self._turtle_state[field][self._last_update_field]:
			return

		self._turtle_state[field][self._data_field] = data
		self._turtle_state[field][self._last_update_field] = time_now


if __name__ == "__main__":
	try:
		T_CONTROL = TurtleControl(RandomMoveStrategy, 2)
		rospy.loginfo("Starting random walker")
		T_CONTROL.run()
	except rospy.ROSInterruptException:
		pass
