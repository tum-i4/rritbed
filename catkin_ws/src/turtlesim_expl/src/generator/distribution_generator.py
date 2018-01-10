#!/usr/bin/env python
""" DistributionGenerator class """

from argument_constraint import ArgumentConstraint

class DistributionGenerator(object):
	""" Container class for distribution parameters """

	_method = None
	name = ""
	args_constraints = []
	rate_in_hz = 0
	queue_size = 0

	def __init__(
		self, method, name, args_constraints=None, rate_in_hz=10, queue_size=10):
		""" Ctor """

		object.__init__(self)

		self._method = method
		self.name = name

		# Default argument count: 2, default values: 0.0 and 1.0
		if args_constraints is None:
			args_constraints = [ArgumentConstraint(0.0), ArgumentConstraint(1.0)]

		self.args_constraints = args_constraints

		self.rate_in_hz = rate_in_hz
		self.queue_size = queue_size


	def generate(self, values=None):
		""" Generate a new value based on the distribution method """

		if values is None:
			values = self.get_default_values()

		args_count = self.get_args_count()

		if len(values) is not args_count:
			raise Exception("Invalid number of values given")

		for i in range(0, args_count):
			if not self.args_constraints[i].fits(values[i]):
				raise Exception("Given value {} does not fit the argument constraint".format(values[i]))

		if args_count is 1:
			return self._method(values[0])
		elif args_count is 2:
			return self._method(values[0], values[1])
		else:
			raise Exception("IMPLEMENTATION MISSING")


	def get_args_count(self):
		return len(self.args_constraints)


	def get_default_values(self):
		return [x.default_value for x in self.args_constraints]
