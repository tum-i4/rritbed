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

	_selected_processor = None


	@staticmethod
	def get_possible_processors():
		""" Get a list of possible processor names. """
		return PosePipe._possible_processors.keys()


	@staticmethod
	def create(*args):
		"""
		Create a PosePipe with a randomly chosen PoseProcessor.
		args: For each possible processor a percentage denoting how likely it will be chosen.\
		Combined must total 100.
		"""

		raise NotImplementedError()


	def __init__(self, processor):
		""" Ctor """

		object.__init__(self)

		self._selected_processor = processor


	def get_processor_name(self):
		""" Get this pipe's processor name. """
		return self._selected_processor.name
