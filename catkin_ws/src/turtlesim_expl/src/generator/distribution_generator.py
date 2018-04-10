#!/usr/bin/env python
""" DistributionGenerator class """

import numpy

from argument_constraint import ArgumentConstraint


class DistributionGenerator(object):
	""" Container class for distribution parameters """

	NORMAL = "normal"
	OFF_VALUE = "off-value"
	HUGE_ERROR = "huge-error"

	LEVELS = ["easy", "med", "hard"]


	def __init__(self,
		method_name, name, args_constraints,
		off_value_values, huge_error_lambdas,
		rate_in_hz=10, queue_size=10):
		""" Ctor """

		object.__init__(self)

		self.name = name
		self.args_constraints = args_constraints
		self.rate_in_hz = rate_in_hz
		self.queue_size = queue_size

		for input_dict in [off_value_values, huge_error_lambdas]:
			if any([l not in input_dict for l in DistributionGenerator.LEVELS]):
				raise ValueError("Given error dictionary is erronous.")

		if any([not isinstance(v, float) for v in off_value_values.values()]):
			raise TypeError("Off-value values are expected to be of float type.")

		if any([not callable(l) for l in huge_error_lambdas.values()]):
			raise TypeError("Huge-error lambdas are expected to be callables.")

		self._off_value_values = off_value_values
		self._huge_error_lambdas = huge_error_lambdas
		self._intrusion_level = None

		self.generate = self._generate_impl

		# pylint: disable-msg=E1101; (Module has no '...' member)
		self.np_rand = numpy.random.RandomState()
		self._method = getattr(self.np_rand, method_name)

		self._intrusion_generators = {
			DistributionGenerator.OFF_VALUE: self._generate_intrusion_off_value,
			DistributionGenerator.HUGE_ERROR: self._generate_intrusion_huge_error
		}


	def activate_intrusion(self, intrusion_mode):
		""" Activate the specified intrusion mode. """

		try:
			self.generate = self._intrusion_generators[intrusion_mode]
		except KeyError:
			raise NotImplementedError("Intrusion mode not implemented")


	def seed(self, seed):
		""" Seed the contained random generator. """
		self.np_rand.seed(seed)


	# pylint: disable-msg=E0202; (Attribute hides this method - intentional)
	def generate(self, values=None):
		""" Generate a new (value, intrusion_str) tuple based on the distribution method. """
		pass


	def _generate_impl(self, values=None):
		""" Implementation for generate() """

		if values is None:
			values = self.get_default_values()

		args_count = self.get_args_count()

		if len(values) is not args_count:
			raise Exception("Invalid number of values given")

		for i in range(0, args_count):
			if not self.args_constraints[i].fits(values[i]):
				raise Exception("Given value {} does not fit the argument constraint".format(values[i]))

		value = 0

		if args_count is 1:
			value = self._method(values[0])
		elif args_count is 2:
			value = self._method(values[0], values[1])
		else:
			raise NotImplementedError("IMPLEMENTATION MISSING")

		return (value, DistributionGenerator.NORMAL)



	### Intrusions ###


	# pylint: disable-msg=W0613; (Unused argument - is necessary)
	def _generate_intrusion_zeroes(self, values=None):
		""" Return zero. """
		return (0, DistributionGenerator.ONLY_ZEROES)


	def _generate_intrusion_huge_error(self, values=None):
		""" Subtract every second generated number by (itself * 100). """

		next_tuple = self._generate_impl(values)

		if self._h_e_last_was_normal:
			error = next_tuple[0] * next_tuple[0]
			multiplier = numpy.random.choice([1, -1])
			next_value = next_tuple[0] + (multiplier * error)
			next_str = DistributionGenerator.HUGE_ERROR
			next_tuple = (next_value, next_str)

		self._h_e_last_was_normal = not self._h_e_last_was_normal

		return next_tuple



	### Getter ###


	def get_args_count(self):
		""" Get the required number of arguments. """
		return len(self.args_constraints)


	def get_default_values(self):
		""" Get the default arguments. """
		return [x.default_value for x in self.args_constraints]
