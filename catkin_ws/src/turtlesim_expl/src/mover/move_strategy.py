#!/usr/bin/env python
""" Move strategy """

class MoveStrategy(object):
	""" Move strategy base class """


	def get_next(self):
		""" Generate next velocity """
		raise NotImplementedError


	def react(self, colour_message):
		""" Generate next velocity by reacting to the colour message """
		raise NotImplementedError
