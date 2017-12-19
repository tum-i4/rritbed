#!/usr/bin/env python
""" Basic mover """

import os
import time
import rospy
from geometry_msgs.msg import Twist

import move_helper


def move():
	""" Move robot according to movement file """

	# Start a new node
	rospy.init_node("robot_mover", anonymous=True)
	velocity_publisher = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

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
		velocity_publisher.publish(vel_msg)
		# TODO: Try cleaner solution
		time.sleep(0.5)

    # Make sure to stop robot after the program has been cancelled
	velocity_publisher.publish(move_helper.get_zero_twist())


if __name__ == "__main__":
	try:
		move()
	except rospy.ROSInterruptException:
		pass
