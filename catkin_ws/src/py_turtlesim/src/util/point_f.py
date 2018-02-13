#!/usr/bin/env python
""" Storage class for a point with two floating point numbers """

from util.point import Point


class PointF(Point):
	""" PointF class """

	# pylint: disable-msg=C0103; (Invalid class attribute name)
	x = 0.0
	y = 0.0

	def __init__(self, x=0, y=0):
		""" Ctor """
		Point.__init__(self)
		self.x = float(x)
		self.y = float(y)


	def update(self, x=None, y=None):
		""" Set the properties to the given values. """

		self.x = float(x) if x is not None else self.x
		self.y = float(y) if y is not None else self.y


	@staticmethod
	def copy(point):
		""" Value-copy the given point. """
		return PointF(point.x, point.y)


	@staticmethod
	def from_point(point):
		""" Create a new PointF instance (value-copy) from the given Point. """
		assert(isinstance(point, Point))
		return PointF.copy(point)


	@staticmethod
	def to_point(point_f):
		""" Create a new Point instance (value-copy) from the given PointF. """
		assert(isinstance(point_f, PointF))
		return Point.copy(point_f)
