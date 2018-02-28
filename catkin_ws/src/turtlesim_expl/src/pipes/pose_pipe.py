#!/usr/bin/env python
""" Module for the PosePipe class """

class PosePipe(object):
	""" Subscribe to a pose topic and process it with the specified PoseProcessor. """

	_possible_processors = ["country code", "poi", "tsp"]


	@staticmethod
	def get_possible_processors():
		""" Get a list of possible processor names. """
		return PosePipe._possible_processors
