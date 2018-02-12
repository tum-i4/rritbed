#!/usr/bin/env python
""" Storage class for a point on a 2D grid """

class Point(object):
	""" Point class """

	# pylint: disable-msg=C0103; (Invalid class attribute name)
	x = 0
	y = 0

	def __init__(self, x=0, y=0):
		""" Ctor """
		object.__init__(self)
		self.x = int(x)
		self.y = int(y)


	def update(self, x=None, y=None):
		""" Set the properties to the given values. """

		self.x = int(x) if x is not None else self.x
		self.y = int(y) if y is not None else self.y


	def __eq__(self, other):
		if type(other) is type(self):
			return self.x == other.x and self.y == other.y
		return False


	def __ne__(self, other):
		return not self.__eq__(other)


	@staticmethod
	def copy(point):
		""" Value-copy the given point. """
		return Point(point.x, point.y)
