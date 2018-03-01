#!/usr/bin/env python
""" Module for the PosePipe class """

import random

from pose_processor import PoseProcessor


class PosePipe(object):
	""" Subscribe to a pose topic and process it with the specified PoseProcessor. """

	CC_STR = "cc"
	POI_STR = "poi"
	TSP_STR = "tsp"
	_possible_processors = {
		CC_STR : None,
		POI_STR : None,
		TSP_STR : None
	}

	_selected_processor = None


	@staticmethod
	def get_possible_processors():
		""" Get a list of possible processor names. """
		return PosePipe._possible_processors.keys()


	@staticmethod
	def create(**kwargs):
		"""
		Create a PosePipe with a randomly chosen PoseProcessor.
		args: For each possible processor a percentage denoting how likely it will be chosen.\
		Combined must total 100.
		"""

		raise NotImplementedError()


	def __init__(self, processor):
		""" Ctor """

		object.__init__(self)

		assert(isinstance(processor, PoseProcessor))
		self._selected_processor = processor


	def get_processor_name(self):
		""" Get this pipe's processor name. """
		return self._selected_processor.name


	def process(self, request):
		""" Process the given request. """
		return self._selected_processor.process(request)
