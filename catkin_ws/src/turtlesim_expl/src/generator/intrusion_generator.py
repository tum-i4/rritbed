#!/usr/bin/env python
""" All generator definitions and corresponding methods """

from distribution_generator import DistributionGenerator

class IntrusionGenerator(DistributionGenerator):
	""" A wrapper for a DistributionGenerator that allows for intrusions """

	ONLY_ZEROES = "zeroes"
	HUGE_ERROR = "huge-error"

	_intrusion_generators = {
		ONLY_ZEROES: _generate_intrusion_zeroes,
		HUGE_ERROR: None
	}

	_generator = None

	def __init__(self, method, name, intrusion_mode,
		args_constraints=None, rate_in_hz=10, queue_size=10):
		""" Ctor """

		DistributionGenerator.__init__(self, method, name, args_constraints, rate_in_hz, queue_size)

		try:
			self.generate = self._intrusion_generators[intrusion_mode]
		except KeyError:
			raise NotImplementedError("Intrusion mode not implemented")


	# pylint: disable-msg=E0202; (Attribute hides this method - intentional)
	def generate(self, values=None):
		""" Generate a new value based on the intrusion-wrapped distribution method """
		# Method is replaced by the respective intrusion generator
		pass


	# pylint: disable-msg=W0613; (Unused argument - is necessary)
	def _generate_intrusion_zeroes(self, values=None):
		""" Hide the generator behind a only-zero-generator """
		return 0
