#!/usr/bin/env python
""" Basic mover """

import os
import time
import rospy
from geometry_msgs.msg import Twist

import move_helper
from turtle_interface import TurtleInterface


class BasicMover(object):
	""" Basic mover based on movement file """

	turtle_if = None
	rate_limiter = None

	def __init__(self):
		""" Ctor """

		object.__init__(self)
		self.turtle_if = TurtleInterface()

		self.rate_limiter = rospy.Rate(2)


	def move(self):
		""" Move robot according to movement file """

		# Used to use os.getcwd()
		file_path = os.path.join(move_helper.BASE_PATH, "move")
		if not os.path.isfile(file_path):
			rospy.logwarn("No movement file found in" + file_path)
			return

		rospy.loginfo("Starting basic movement with movement file " + file_path)

		current_line = 0
		file_contents = []

		# Try to read the whole movement file
		try:
			file_reader = open(file_path)
			file_contents = file_reader.readlines()
			file_reader.close()
		except IOError:
			rospy.logwarn("Couldn't read movement file")
			return

		# While loop to assure that Ctrl-C can exit the app
		while not rospy.is_shutdown():
			vel_msg = move_helper.get_zero_twist()

			if current_line >= len(file_contents):
				rospy.loginfo("End of movement file reached")
				return

			# Read next movement command
			next_line = file_contents[current_line]
			current_line += 1

			# Remove trailing new line
			newline_index = len(next_line) - 1
			assert(next_line[newline_index:] == "\n")
			next_line = next_line[:newline_index]

			try:
				vel_msg = move_helper.get_twist_from_string(next_line)
			except ValueError:
				rospy.logwarn("Invalid value read from line:\n" + next_line)
				return

			# We have read the velocity and can now publish it
			self.turtle_if.publish(vel_msg)

			self.rate_limiter.sleep()

		# Make sure to stop robot after the program has been cancelled
		self.turtle_if.publish(move_helper.get_zero_twist())


if __name__ == "__main__":
	try:
		T_MOVER = BasicMover()
		T_MOVER.move()
	except rospy.ROSInterruptException:
		pass
