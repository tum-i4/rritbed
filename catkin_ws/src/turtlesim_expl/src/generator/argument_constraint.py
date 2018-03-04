#!/usr/bin/env python
""" ArgumentConstraint class """

from sys import maxint as MAXINT


class ArgumentConstraint(object):
	""" Define constraints for an argument (min, max) """

	def __init__(self, default_value, min_value=-MAXINT-1, max_value=MAXINT):
		""" Ctor """

		object.__init__(self)

		if min_value > max_value:
			raise ValueError("min <= max")

		self.default_value = float(default_value)
		self.min_value = float(min_value)
		self.max_value = float(max_value)

		if not self.fits(default_value):
			raise ValueError("Default value must fit the constraint")


	def fits(self, value):
		""" Checks if the given value fits """
		return value >= self.min_value and value <= self.max_value
