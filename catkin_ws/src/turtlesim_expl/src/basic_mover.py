#!/usr/bin/env python
""" Basic mover """

import os
import time
import rospy
from geometry_msgs.msg import Twist

PI = 3.1415926535897
MOVE_FILE_PATH = "/ros"


def move():
	""" Move robot according to movement file """

	# Start a new node
	rospy.init_node("robot_mover", anonymous=True)
	velocity_publisher = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

	# Used to use os.getcwd()
	file_path = os.path.join(MOVE_FILE_PATH, "move")
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
		vel_msg = get_zero_twist()

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
			vel_msg = get_twist_from_string(next_line)
		except ValueError:
			rospy.logwarn("Invalid value read from line:\n" + next_line)
			return

		# We have read the velocity and can now publish it
		velocity_publisher.publish(vel_msg)
		# TODO: Try cleaner solution
		time.sleep(0.5)

    # Make sure to stop robot after the program has been cancelled
	velocity_publisher.publish(get_zero_twist())


def get_zero_twist():
	""" Return new twist initialised to zero """

	new_twist = Twist()

	new_twist.linear.x = 0
	new_twist.linear.y = 0
	new_twist.linear.z = 0
	new_twist.angular.x = 0
	new_twist.angular.y = 0
	new_twist.angular.z = 0

	return new_twist


def get_twist_from_string(value_string):
	""" Return new twist initialised to the values given in the string """

	value_array = value_string.split(",")

	if not len(value_array) == 6:
		raise ValueError("Incorrect number of elements in line")

	# Might throw exception - intentional!
	value_array = [int(x) for x in value_array]

	new_twist = Twist()

	new_twist.linear.x = value_array[0]
	new_twist.linear.y = value_array[1]
	new_twist.linear.z = value_array[2]
	new_twist.angular.x = value_array[3]
	new_twist.angular.y = value_array[4]
	new_twist.angular.z = value_array[5]

	return new_twist


if __name__ == "__main__":
	try:
		move()
	except rospy.ROSInterruptException:
		pass
