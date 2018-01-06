#!/usr/bin/env python
""" Move helper functions """

from geometry_msgs.msg import Twist


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
	"""
	Return new twist initialised to the values given in the string
	value_string: Should be formatted as "i,i,i,i,i,i" with six integer values
	"""

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
