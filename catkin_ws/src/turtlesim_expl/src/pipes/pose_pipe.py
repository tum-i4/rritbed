#!/usr/bin/env python
""" Module for the PosePipe class """

import random

from pose_processor import PoseProcessor, CC_STR, POI_STR, TSP_STR


class PosePipe(object):
	""" Subscribe to a pose topic and process it with the specified PoseProcessor. """

	_POSSIBLE_PROCESSORS = {
		CC_STR : PoseProcessor(CC_STR),
		POI_STR : PoseProcessor(POI_STR),
		TSP_STR : PoseProcessor(TSP_STR)
	}


	@staticmethod
	def get_possible_processors():
		""" Get a list of possible processor names. """
		return PosePipe._POSSIBLE_PROCESSORS.keys()


	@staticmethod
	def create(intrusion, intrusion_field, **kwargs):
		"""
		Create a PosePipe with a randomly chosen PoseProcessor.
		kwargs: For each possible processor a value denoting how likely it will be chosen.\
		Must be positive integers.
		"""

		if(len(kwargs) != len(PosePipe._POSSIBLE_PROCESSORS)):
			raise ValueError("Invalid number of arguments given!")

		if any([not isinstance(x, int) or x < 0 for x in kwargs.values()]):
			raise ValueError("All arguments must be positive integers.")

		# Ensure each key is present in the dict
		for key in kwargs:
			if key not in PosePipe._POSSIBLE_PROCESSORS.keys():
				raise KeyError("Given key [{}] not found".format(key))

		choices = []
		for name, likelihood in kwargs.items():
			choices += [name] * likelihood
		choice = random.choice(choices)

		processor = PosePipe._POSSIBLE_PROCESSORS[choice]
		processor.set_intrusion(intrusion, intrusion_field)

		return PosePipe(processor)


	def __init__(self, processor):
		""" Ctor """

		object.__init__(self)

		assert(isinstance(processor, PoseProcessor))
		self._selected_processor = processor


	def get_processor_name(self):
		""" Get this pipe's processor name. """
		return self._selected_processor.name


	def process(self, request, label):
		""" Process the given request. """
		return self._selected_processor.process(request, label=label)



if __name__ == "__main__":
	PP = PosePipe.create(intrusion=None, intrusion_field="INVALID", cc=50, poi=25, tsp=25)
	print("Possible processors: {}".format(PP.get_possible_processors()))
	print("Processor name: {}".format(PP.get_processor_name()))
	print("Process [x: 5] [y: 10]: {}".format(
		PP.process(PoseProcessor.add_to_request({}, 5, 10), label=True)))
