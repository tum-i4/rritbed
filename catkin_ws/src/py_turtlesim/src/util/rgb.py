#!/usr/bin/env python
""" Storage class for RGB information """

# pylint: disable-msg=C0103; (Invalid attribute / argument names r, g, b)


class Rgb(object):
	""" RGB class """

	MIN = 0
	MAX = 255

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
		""" Return a pastel purple Rgb object. """
		return Rgb(r=150, g=140, b=200)


	@staticmethod
	def pastel_yellow():
		""" Return a pastel yellow Rgb object. """
		return Rgb(r=170, g=250, b=140)


	@staticmethod
	def pastel_green():
		""" Return a pastel green Rgb object. """
		return Rgb(r=120, g=180, b=130)


	@staticmethod
	def pastel_blue():
		""" Return a pastel blue Rgb object. """
		return Rgb(r=120, g=180, b=200)


	@staticmethod
	def strong_red():
		""" Return a red Rgb object. """
		return Rgb(r=Rgb.MAX)


	@staticmethod
	def med_red():
		""" Return a medium red Rgb object. """
		return Rgb(r=200, g=50, b=50)


	@staticmethod
	def light_red():
		""" Return a light red Rgb object. """
		return Rgb(r=170, g=80, b=80)
