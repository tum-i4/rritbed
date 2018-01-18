#!/usr/bin/env python
""" Move strategy """

class MoveStrategy(object):
	""" Move strategy base class """


	def get_next(self):
		""" Generate next velocity """
		raise NotImplementedError
