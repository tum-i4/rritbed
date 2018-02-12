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
		self.x = x
		self.y = y
