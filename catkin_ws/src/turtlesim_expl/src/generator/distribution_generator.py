#!/usr/bin/env python
""" DistributionGenerator class """

class DistributionGenerator(object):
	""" Container class for distribution parameters """

	_method = None
	name = ""
	args_count = 0
	default_values = []
	rate_in_hz = 0
	queue_size = 0

	def __init__(
		self, method, name, args_count=2, default_values=None, rate_in_hz=10, queue_size=10):
		""" Ctor """

		object.__init__(self)

		self._method = method
		self.name = name
		self.args_count = args_count

		if default_values is None:
			default_values = [0.0, 1.0]

		self.default_values = default_values
		self.rate_in_hz = rate_in_hz
		self.queue_size = queue_size


	def generate(self, values=None):
		""" Generate a new value based on the distribution method """

		if values is None:
			values = self.default_values

		if len(values) is not self.args_count:
			raise Exception("Invalid number of values given")

		if self.args_count is 1:
			return self._method(values[0])
		elif self.args_count is 2:
			return self._method(values[0], values[1])
		else:
			raise Exception("IMPLEMENTATION MISSING")
