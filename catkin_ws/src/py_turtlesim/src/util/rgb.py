#!/usr/bin/env python
""" Storage class for RGB information """

class Rgb(object):
	""" RGB class """

	MIN = 0
	MAX = 255

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


	def __eq__(self, other):
		if type(other) is type(self):
			return self.r == other.r and self.g == other.g and self.b == other.b
		return False


	def __ne__(self, other):
		return not self.__eq__(other)


	@staticmethod
	def pastel_purple():
		""" Return a pastel purple Rgb object """
		return Rgb(r=150, g=125, b=210)


	@staticmethod
	def pastel_yellow():
		""" Return a pastel yellow Rgb object """
		return Rgb(r=255, g=240, b=120)


	@staticmethod
	def pastel_green():
		""" Return a pastel green Rgb object """
		return Rgb(r=100, g=180, b=100)


	@staticmethod
	def pastel_blue():
		""" Return a pastel blue Rgb object """
		return Rgb(r=100, g=180, b=250)


	@staticmethod
	def red():
		""" Return a red Rgb object """
		return Rgb(r=Rgb.MAX)
