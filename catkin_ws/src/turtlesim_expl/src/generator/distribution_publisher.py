#!/usr/bin/env python
"""
Create a JSON file describing all possible generators
Usage:
--generators [file name]

Generate data based on a distribution and publish it to ROS
Possible arguments:
gaussian [loc] [scale] : Normal distribution
gumbel [loc] [scale]
laplace [loc] [scale]
logistic [loc] [scale] : Logistic distribution - scale > 0
pareto [alpha]         : Pareto distribution - alpha > 0
rayleigh [scale]       : Rayleigh distribution - scale >= 0
uniform [low] [high]   : Uniform distribution - low < high
vonmises [mu] [kappa]  : Von Mises distribution - kappa >= 0
wald [mean] [scale]    : Wald distribution - mean > 0, scale >= 0
weibull [a]            : Weibull distribution - a > 0
zipf [a]               : Zipf distribution - a > 1

Publish distribution based on pre-generated file
Possible arguments:
file <file name> [-r]  : Repeat after reaching EOF
"""

# pylint: disable-msg=C1801

import os
import sys
import numpy as np
import rospy
from std_msgs.msg import Float32

from distribution_generator import DistributionGenerator

_BASE_PATH = "~/ros"


class DistributionPublisher(object):
	""" Data generation class based on distribution """

	_base_path_expanded = ""

	_publisher = None

	_file_based = False

	_file_contents = []
	_current_line = 0
	_repeat_file = False

	_sub_routine = ""

	_generator_arguments = []


	_gaussian_str = "gaussian"
	_gumbel_str = "gumbel"
	_laplace_str = "laplace"
	_logistic_str = "logistic"
	_pareto_str = "pareto"
	_rayleigh_str = "rayleigh"
	_uniform_str = "uniform"
	_vonmises_str = "vonmises"
	_wald_str = "wald"
	_weibull_str = "weibull"
	_zipf_str = "zipf"

	_file_based_str = "file"

	# pylint: disable-msg=E1101
	_generators = {
		# Gaussian, Gumbel, Laplace: loc and scale arbitrary
		_gaussian_str : DistributionGenerator(np.random.normal, _gaussian_str),
		_gumbel_str : DistributionGenerator(np.random.gumbel, _gumbel_str),
		_laplace_str : DistributionGenerator(np.random.laplace, _laplace_str),
		# Logistic: loc arbitrary, scale > 0
		_logistic_str : DistributionGenerator(np.random.logistic, _logistic_str),
		# Pareto: a(lpha) > 0
		_pareto_str : DistributionGenerator(np.random.pareto, _pareto_str, 1, [1.0]),
		# Rayleigh: scale > 0
		_rayleigh_str : DistributionGenerator(np.random.rayleigh, _rayleigh_str, 1, [1.0]),
		# Uniform: low < high
		_uniform_str : DistributionGenerator(np.random.uniform, _uniform_str),
		# Von Mises: mu arbitrary, kappa >= 0
		_vonmises_str : DistributionGenerator(np.random.vonmises, _vonmises_str),
		# Wald: mean > 0, scale > 0
		_wald_str : DistributionGenerator(np.random.wald, _wald_str, 2, [1.0, 1.0]),
		# Weibull: a > 0
		_weibull_str : DistributionGenerator(np.random.weibull, _weibull_str, 1, [5.0]),
		# Zipf: a > 1
		_zipf_str : DistributionGenerator(np.random.zipf, _zipf_str, 1, [2.0])
	}


	def __init__(self):
		""" Ctor """

		object.__init__(self)

		self._base_path_expanded = os.path.expanduser(_BASE_PATH)

		# Remove remapping arguments
		args = rospy.myargv(sys.argv)

		# Delete program name from arguments
		del args[0]

		if not args:
			raise Exception("No arguments given")

		# 1) JSON dump mode
		# --generators [file path]

		if args[0] == "--generators":
			file_path = args[1] if len(args) == 2 else None
			self._generator_mode(file_path)
			exit()

		# 2) Publishing mode

		name = args[0]
		queue_size = 10

		#    a) File based
		if args[0] == self._file_based_str:
			self._setup_reader(args)
			name += "_" + args[1]
		#    b) Generator based
		else:
			self._setup_generator(args)
			queue_size = self._generators[self._sub_routine].queue_size

		rospy.init_node(name, anonymous=True)
		self._publisher = rospy.Publisher(name, Float32, queue_size=queue_size)


	def _setup_reader(self, my_args):
		"""
		Setup for file-based publishing
		my_args: Arguments trimmed to <file_name> [-r]
		"""

		if len(my_args) == 1:
			raise Exception("No file name given")

		if len(my_args) > 3:
			raise Exception("Too many arguments supplied: {}".format(my_args[1:]))

		self._file_based = True

		# File path argument
		file_path = my_args[1]

		# Either a full path was given (contains sep), otherwise the name is appended to the default path
		if os.sep not in file_path:
			file_path = os.path.join(self._base_path_expanded, "data", file_path)

		if not os.path.isfile(file_path):
			raise Exception("No file found at {}".format(file_path))

		try:
			file_reader = open(file_path)
			self._file_contents = file_reader.readlines()
			file_reader.close()
		except IOError:
			raise Exception("Couldn't read file {}".format(file_path))

		# Repeat file argument
		if len(my_args) > 2 and my_args[2] == "-r":
			self._repeat_file = True


	def _setup_generator(self, my_args):
		"""
		Setup for data generation
		my_args: Arguments trimmed to <generator> [default arg] ([default arg 2] ...)
		"""

		try:
			generator = self._generators[my_args[0]]
		except KeyError:
			raise Exception("Could not find specified sub-routine {}".format(my_args[0]))

		self._sub_routine = my_args[0]
		generator_arguments = my_args[1:]

		# pylint: disable-msg=W1202
		if len(generator_arguments) != generator.args_count:
			self._generator_arguments = generator.default_values
			rospy.loginfo("Initialising with default values %s", self._generator_arguments)
			return

		self._generator_arguments = [float(x) for x in generator_arguments]
		rospy.loginfo("Initialising with values %s", self._generator_arguments)


	def run(self):
		""" Run the distribution publisher with the given rate limiter and create num method """

		rate_in_hz = 10
		create_num = self._read

		if not self._file_based:
			rate_in_hz = self._generators[self._sub_routine].rate_in_hz
			create_num = self._generate

		rate_limiter = rospy.Rate(rate_in_hz)

		# While loop to assure that Ctrl-C can exit the app
		while not rospy.is_shutdown():
			next_num = create_num()

			if next_num is None:
				break

			rospy.loginfo("Value: %s", next_num)
			self._publisher.publish(next_num)
			rate_limiter.sleep()


	def _read(self):
		"""
		Read next line from file
		Returns: Either repeats based on self._repeat_file or None if EOF is reached
		"""

		if self._current_line >= len(self._file_contents):
			if self._repeat_file:
				self._current_line = 0
			else:
				rospy.loginfo("End of data file reached")
				return None

		next_num = float(self._file_contents[self._current_line])
		self._current_line += 1
		return next_num


	def _generate(self):
		""" Generate data with current generator """
		return self._generators[self._sub_routine].generate()


	def _generator_mode(self, file_path):
		""" Print all current generator definitions out (file_path is None) or write them to a file """

		raise Exception("IMPLEMENT")


if __name__ == "__main__":
	try:
		PUB = DistributionPublisher()
		PUB.run()
	except rospy.ROSInterruptException:
		pass
