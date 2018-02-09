#!/usr/bin/env python
""" All generator definitions and corresponding methods """

from distribution_generator import DistributionGenerator

class IntrusionGenerator(DistributionGenerator):
	""" A wrapper for a DistributionGenerator that allows for intrusions """

	ONLY_ZEROES = "zeroes"
	HUGE_ERROR = "huge-error"

	test = 0

	def __init__(self, method, name, args_constraints=None, rate_in_hz=10, queue_size=10):
		""" Ctor """

		DistributionGenerator.__init__(self, method, name, args_constraints, rate_in_hz, queue_size)

		raise NotImplementedError()
