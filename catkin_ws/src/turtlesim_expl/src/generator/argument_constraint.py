#!/usr/bin/env python
""" ArgumentConstraint class """

from sys import maxint as MAXINT


class ArgumentConstraint(object):
	""" Define constraints for an argument (min, max) """

	default_value = None
	min_value = 0.0
	max_value = 0.0

	def __init__(self, default_value=None, min_value=-MAXINT-1, max_value=MAXINT):
		""" Ctor """

		object.__init__(self)

		if min_value > max_value:
			raise ValueError("min <= max")

		if default_value is not None:
			self.default_value = default_value * 1.0

		self.min_value = min_value * 1.0
		self.max_value = max_value * 1.0
