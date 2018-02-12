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


	def pastel_purple(self):
		""" Return a pastel purple colour """
		return Rgb(r=150, g=125, b=210)


	def pastel_yellow(self):
		""" Return a pastel yellow colour """
		return Rgb(r=255, g=240, b=120)


	def pastel_green(self):
		""" Return a pastel green colour """
		return Rgb(r=100, g=180, b=100)


	def pastel_blue(self):
		""" Return a pastel blue colour """
		return Rgb(r=100, g=180, b=250)


	def red(self):
		""" Return a red colour """
		return Rgb(r=Rgb.MAX)
