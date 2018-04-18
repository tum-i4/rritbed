#!/usr/bin/env python
""" DistributionGenerator class """

from collections import namedtuple
import numpy


class DistributionGenerator(object):
	""" Container class for distribution parameters """

	NORMAL = "normal"
	OFF_VALUE = "off-value"
	HUGE_ERROR = "huge-error"

	LEVELS = ["easy", "med", "hard"]


	def __init__(self,
		method_name, name, args_constraints, expected_range, mean,
		rate_in_hz=2, queue_size=10):
		""" Ctor """

		object.__init__(self)

		self.name = name
		self.args_constraints = args_constraints
		self.rate_in_hz = rate_in_hz
		self.queue_size = queue_size

		if not isinstance(expected_range, list) or len(expected_range) != 2:
			raise TypeError("expected_values is supposed to be a list with two values [min, max].")

		if not isinstance(mean, float):
			raise TypeError("mean expects a float value")

		self._expected_range = expected_range
		self._mean = mean
		self._set_intrusion_parameters()
		self._intrusion_level = None

		self.generate = self._generate_impl

		# pylint: disable-msg=E1101; (Module has no '...' member)
		self.np_rand = numpy.random.RandomState()
		self._method = getattr(self.np_rand, method_name)

		self._intrusion_generators = {
			DistributionGenerator.OFF_VALUE: self._generate_intrusion_off_value,
			DistributionGenerator.HUGE_ERROR: self._generate_intrusion_huge_error
		}


	def _set_intrusion_parameters(self):
		""" Calculate the off_value values based on the given expected min/max values. """

		min_val = min(self._expected_range)
		max_val = max(self._expected_range)
		assert(min_val < self._mean)
		assert(max_val > self._mean)

		# span = max_val - min_val
		span_l = self._mean - min_val
		span_r = max_val - self._mean
		# base = (max_val - min_val) / float(2)

		if len(DistributionGenerator.LEVELS) != 3:
			raise NotImplementedError("Expected three levels")

		easy_factor = 5
		med_factor = 1.5
		hard_factor = 1.001

		err_tuple = namedtuple("err_tuple", "l r")
		easy_err = err_tuple(span_l * easy_factor, span_r * easy_factor)
		med_err = err_tuple(span_l * med_factor, span_r * med_factor)
		hard_err = err_tuple(span_l * hard_factor, span_r * hard_factor)

		self._errors = {
			DistributionGenerator.LEVELS[0] : easy_err,
			DistributionGenerator.LEVELS[1] : med_err,
			DistributionGenerator.LEVELS[2] : hard_err
		}

		self._off_value_values = {
			DistributionGenerator.LEVELS[0] :
				[self._mean - easy_err.l, self._mean + easy_err.r],
			DistributionGenerator.LEVELS[1] :
				[self._mean - med_err.l, self._mean + med_err.r],
			DistributionGenerator.LEVELS[2] :
				[self._mean - hard_err.l, self._mean + hard_err.r],
		}


	def activate_intrusion(self, intrusion_mode, intrusion_level):
		""" Activate the specified intrusion mode. """

		if intrusion_level not in DistributionGenerator.LEVELS:
			raise ValueError("Given intrusion level is invalid: {}".format(intrusion_level))

		self._intrusion_level = intrusion_level

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
	def _generate_intrusion_off_value(self, values=None):
		""" Return the off-value of the selected intrusion level. """

		choices = self._off_value_values[self._intrusion_level]
		off_value = numpy.random.choice(choices)

		return (off_value, DistributionGenerator.OFF_VALUE)


	def _generate_intrusion_huge_error(self, values=None):
		""" Calculate a huge error based on the selected intrusion level. """

		normal_value, _ = self._generate_impl(values)
		factor = -1
		error_span = self._errors[self._intrusion_level].l
		if normal_value >= self._mean:
			factor = 1
			error_span = self._errors[self._intrusion_level].r

		# Generated value < 0: take span_l, subtract from mean
		error_level = self._mean + factor * error_span
		next_value = error_level + factor * (normal_value * normal_value)

		return (next_value, DistributionGenerator.HUGE_ERROR)



	### Getter ###


	def get_args_count(self):
		""" Get the required number of arguments. """
		return len(self.args_constraints)


	def get_default_values(self):
		""" Get the default arguments. """
		return [x.default_value for x in self.args_constraints]
