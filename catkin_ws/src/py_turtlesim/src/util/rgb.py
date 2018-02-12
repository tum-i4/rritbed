#!/usr/bin/env python
""" Storage class for RGB information """

class Rgb(object):
	""" RGB class """

	# pylint: disable-msg=C0103; (Invalid class attribute name)
	r = 0
	g = 0
	b = 0

	def __init__(self, r=0, g=0, b=0):
		""" Ctor """
		object.__init__(self)
		self.r = r
		self.g = g
		self.b = b
