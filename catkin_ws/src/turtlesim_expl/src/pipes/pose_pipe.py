#!/usr/bin/env python
""" Module for the PosePipe class """

import random

from pipes.pose_processor import PoseProcessor, CC_STR, POI_STR, TSP_STR


class PosePipe(object):
	""" Subscribe to a pose topic and process it with the specified PoseProcessor. """

	_possible_processors = {
		CC_STR : PoseProcessor(CC_STR),
		POI_STR : PoseProcessor(POI_STR),
		TSP_STR : PoseProcessor(TSP_STR)
	}

	_selected_processor = None


	@staticmethod
	def get_possible_processors():
		""" Get a list of possible processor names. """
		return PosePipe._possible_processors.keys()


	@staticmethod
	def create(intrusion, intrusion_field, **kwargs):
		"""
		Create a PosePipe with a randomly chosen PoseProcessor.
		kwargs: For each possible processor a percentage denoting how likely it will be chosen.\
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
	PP = PosePipe.create(intrusion=None, cc=50, poi=25, tsp=25)
	print("Possible processors: {}".format(PP.get_possible_processors()))
	print("Processor name: {}".format(PP.get_processor_name()))
	print("Process [x: 5] [y: 10]: {}".format(PP.process(PoseProcessor.add_to_request({}, 5, 10))))
