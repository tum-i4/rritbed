#!/usr/bin/env python
""" Module for the PosePipe class """

class PosePipe(object):
	""" Subscribe to a pose topic and process it with the specified PoseProcessor. """

	_cc_str = "country code"
	_poi_str = "poi"
	_tsp_str = "tsp"
	_possible_processors = {
		_cc_str : None,
		_poi_str : None,
		_tsp_str : None
	}


	@staticmethod
	def get_possible_processors():
		""" Get a list of possible processor names. """
		return PosePipe._possible_processors.keys()
