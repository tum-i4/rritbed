#!/usr/bin/env python
""" ArgumentConstraint class """

from sys import maxint as MAXINT

class ArgumentsConstraints(object):
	""" Define constraints for arguments (number, and constraint for each argument) """

	number_of_arguments = 0
	argument_constraints = []

	def __init__(self, number_of_arguments, argument_constraints):
		""" Ctor """

		object.__init__(self)

		if number_of_arguments != len(argument_constraints):
			raise Exception("Invalid number of argument constraints supplied")

		self.number_of_arguments = number_of_arguments
		self.argument_constraints = argument_constraints




class ArgumentConstraint(object):
	""" Define constraints for an argument (min, max) """

	min_value = 0
	max_value = 0
	default_value = None

	def __init__(self, default_value=None, min_value=-MAXINT-1, max_value=MAXINT):
		""" Ctor """

		object.__init__(self)

		if min_value > max_value:
			raise ValueError("min <= max")

		self.min_value = min_value or self.min_value
		self.max_value = max_value or self.max_value

		self.default_value = default_value
