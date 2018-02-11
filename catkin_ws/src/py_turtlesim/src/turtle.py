#!/usr/bin/env python
"""
Rebuilding turtle.cpp in Python
"""

DEFAULT_PEN_R = 0xb3
DEFAULT_PEN_G = 0xb8
DEFAULT_PEN_B = 0xff

class Turtle(object):
	""" The turtle class """

	def __init__(self):
		""" Ctor """

		object.__init__(self)
