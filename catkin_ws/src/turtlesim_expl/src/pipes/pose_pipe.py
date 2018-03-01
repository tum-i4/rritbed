#!/usr/bin/env python
""" Module for the PosePipe class """

import random

from pose_processor import PoseProcessor, CC_STR, POI_STR, TSP_STR


class PosePipe(object):
	""" Subscribe to a pose topic and process it with the specified PoseProcessor. """

	_possible_processors = {
		CC_STR : PoseProcessor(CC_STR, PoseProcessor.process_cc),
		POI_STR : PoseProcessor(POI_STR, PoseProcessor.process_poi),
		TSP_STR : PoseProcessor(TSP_STR, PoseProcessor.process_tsp)
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

		assert(len(kwargs) == len(PosePipe._possible_processors))
		percentages = [int(x) for x in kwargs.values()]
		assert(sum(percentages) == 100)
		# Ensure each key is present in the dict
		for key in kwargs:
			# pylint: disable-msg=W0104; (Statement has no effect)
			PosePipe._possible_processors[key]

		choices = []

		for name, percentage in kwargs.items():
			choices += [name] * percentage

		choice = random.choice(choices)
		processor = PosePipe._possible_processors[choice]

		return PosePipe(processor)


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



if __name__ == "__main__":
	PP = PosePipe.create(cc=50, poi=25, tsp=25)
	print("Possible processors: %s", PP.get_possible_processors())
	print("Processor name: %s", PP.get_processor_name())
	print("Process x5, y10: %s", PP.process(PoseProcessor.add_to_request({}, 5, 10)))
