#!/usr/bin/env python
""" Basic move strategy """

import os
import rospy

import move_helper
from move_strategy import MoveStrategy
from turtle_control import TurtleControl

BASE_PATH = "/tmp/ros"


class BasicMoveStrategy(MoveStrategy):
	""" Basic move strategy based on movement file """

	_file_path = ""
	_current_line = 0


	def __init__(self):
		""" Ctor """
		MoveStrategy.__init__(self)


	def get_next(self):
		""" Get next velocity from movement file """

		# Used to use os.getcwd()
		self._file_path = os.path.join(BASE_PATH, "move")
		if not os.path.isfile(self._file_path):
			rospy.logwarn("No movement file found in" + self._file_path)
			return None

		self._current_line = 0
		file_contents = []

		# Try to read the whole movement file
		try:
			file_reader = open(self._file_path)
			file_contents = file_reader.readlines()
			file_reader.close()
		except IOError:
			rospy.logwarn("Couldn't read movement file")
			return None

		vel_msg = move_helper.get_zero_twist()

		if self._current_line >= len(file_contents):
			rospy.loginfo("End of movement file reached")
			return None

		# Read next movement command
		next_line = file_contents[self._current_line]
		self._current_line += 1

		# Remove trailing new line
		newline_index = len(next_line) - 1
		assert(next_line[newline_index:] == "\n")
		next_line = next_line[:newline_index]

		try:
			vel_msg = move_helper.get_twist_from_string(next_line)
		except ValueError:
			rospy.logerr("Invalid value read from line:\n" + next_line)
			return None

		# Return read velocity
		return vel_msg


if __name__ == "__main__":
	try:
		T_CONTROL = TurtleControl(BasicMoveStrategy, 2)
		rospy.loginfo("Starting file-based mover")
		T_CONTROL.run()
	except rospy.ROSInterruptException:
		pass
