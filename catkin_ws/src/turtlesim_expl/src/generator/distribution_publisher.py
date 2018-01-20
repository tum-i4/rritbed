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


import argparse
import json
import os
import sys
import numpy as np
import rospy
from std_msgs.msg import Float32

from distribution_generator import DistributionGenerator
from argument_constraint import ArgumentConstraint as AC

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


	# pylint: disable-msg=E1101
	_generators = {
		# Gaussian, Gumbel, Laplace: loc and scale arbitrary
		_gaussian_str : DistributionGenerator(np.random.normal, _gaussian_str),
		_gumbel_str : DistributionGenerator(np.random.gumbel, _gumbel_str),
		_laplace_str : DistributionGenerator(np.random.laplace, _laplace_str),
		# Logistic: loc arbitrary, scale > 0
		_logistic_str : DistributionGenerator(np.random.logistic, _logistic_str,
			[AC(0.0), AC(1.0, min_value=0.1)]),
		# Pareto: a(lpha) > 0
		_pareto_str : DistributionGenerator(np.random.pareto, _pareto_str,
			[AC(1.0, min_value=0.1)]),
		# Rayleigh: scale > 0
		_rayleigh_str : DistributionGenerator(np.random.rayleigh, _rayleigh_str,
			[AC(1.0, min_value=0.1)]),
		# Uniform: low < high (not a binding constraint)
		_uniform_str : DistributionGenerator(np.random.uniform, _uniform_str),
		# Von Mises: mu arbitrary, kappa >= 0
		_vonmises_str : DistributionGenerator(np.random.vonmises, _vonmises_str,
			[AC(0.0), AC(1.0, min_value=0)]),
		# Wald: mean > 0, scale > 0
		_wald_str : DistributionGenerator(np.random.wald, _wald_str,
			[AC(1.0, min_value=0.1), AC(1.0, min_value=0.1)]),
		# Weibull: a > 0
		_weibull_str : DistributionGenerator(np.random.weibull, _weibull_str,
			[AC(5.0, min_value=0.1)]),
		# Zipf: a > 1
		_zipf_str : DistributionGenerator(np.random.zipf, _zipf_str,
			[AC(2.0, min_value=1.1)])
	}


	def __init__(self):
		""" Ctor """

		object.__init__(self)

		self._base_path_expanded = os.path.expanduser(_BASE_PATH)

		generator_choices = []
		for key in self._generators:
			generator_choices.append(key)

		parser = argparse.ArgumentParser(prog="dist_pub")
		sub_parsers = parser.add_subparsers(title="modes", dest="mode")

		# Live gen mode
		parser_gen = sub_parsers.add_parser("gen", help="Generate data live from a distribution")
		parser_gen.add_argument("generator", choices=generator_choices,
			help="The generator name")
		parser_gen.add_argument("params", type=float, nargs="*", help="Optional parameters")

		# File pub mode
		parser_file = sub_parsers.add_parser("file", help="Publish data from a file")
		parser_file.add_argument("pub_file_path", metavar="/FILE/PATH", help="The file to publish from")
		parser_file.add_argument("--repeat", "-r", action="store_true", dest="repeat_file")

		# Gen defs mode
		parser_defs = sub_parsers.add_parser("defs",
			help="Print out or save generator definitions to file")
		parser_defs.add_argument("defs_file_path", nargs="?", metavar="/FILE/PATH", default=None,
			help="File path to store the definitions to")

		# Remove remapping arguments and delete program name from arguments
		filtered_argv = rospy.myargv(sys.argv)[1:]

		args = parser.parse_args(filtered_argv)

		# 1) JSON dump mode

		if args.mode == "defs":
			self._generator_mode(args.defs_file_path)
			exit()

		# 2) Publishing mode

		name = ""
		queue_size = 10
		return_message = ""

		#    a) File based
		if args.mode == "file":
			return_message = self._setup_reader(args.pub_file_path, args.repeat_file)
			name = "file_" + os.path.basename(args.pub_file_path)
		#    b) Generator based
		elif args.mode == "gen":
			return_message = self._setup_generator(args.generator, args.params)
			name = args.generator
			queue_size = self._generators[self._sub_routine].queue_size
		else:
			raise NotImplementedError

		rospy.init_node(name, anonymous=True)
		rospy.loginfo(return_message)
		self._publisher = rospy.Publisher(name, Float32, queue_size=queue_size)


	def _setup_reader(self, file_path, repeat_file):
		"""
		Setup for file-based publishing
		"""

		self._file_based = True

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
		self._repeat_file = repeat_file

		return "Publishing file-based from {}{}".format(
			file_path, " (repeating)" if self._repeat_file else "")


	def _setup_generator(self, gen_name, parameters):
		"""
		Setup for data generation
		"""

		try:
			generator = self._generators[gen_name]
		except KeyError:
			raise Exception("Could not find specified sub-routine {}".format(gen_name))

		self._sub_routine = gen_name

		# pylint: disable-msg=W1202
		if len(parameters) != generator.get_args_count():
			self._generator_arguments = generator.get_default_values()
			return "Initialising with default values {}".format(self._generator_arguments)

		self._generator_arguments = [float(x) for x in parameters]
		return "Initialising with values {}".format(self._generator_arguments)


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

		if file_path is not None:
			self._generator_mode_write_to_file(file_path)
			exit()

		print("name, ['default, min, max', ...]")
		for gen_name in self._generators:
			constraints = self._generators[gen_name].args_constraints
			print("{}, {}".format(gen_name, ["{}, {}, {}".format(
				x.default_value, x.min_value, x.max_value)
				for x in constraints]))


	def _generator_mode_write_to_file(self, file_path):

		file_path = os.path.expanduser(file_path)

		folder_path = os.path.dirname(file_path)
		if not os.path.exists(folder_path):
			print("File path {} does not exist".format(folder_path))
			exit()

		if os.path.exists(file_path):
			print("File {} exists and would be overwritten".format(file_path))
			exit()

		gen_defs = {}

		for gen_name in self._generators:
			constraints = self._generators[gen_name].args_constraints
			gen_defs[gen_name] = [
				{"default" : x.default_value, "min" : x.min_value, "max" : x.max_value}
				for x in constraints]

		result = json.dumps(gen_defs)

		with open(file_path, 'w') as file_writer:
			file_writer.write(result)

		print("Data written to file {} successfully".format(file_path))


if __name__ == "__main__":
	try:
		PUB = DistributionPublisher()
		PUB.run()
	except rospy.ROSInterruptException:
		pass
