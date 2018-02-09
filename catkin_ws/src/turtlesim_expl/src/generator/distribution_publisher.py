#!/usr/bin/env python
"""
Generate data based on a distribution and publish it to ROS
Possible arguments:
gen gaussian [loc] [scale] : Normal distribution
gen gumbel [loc] [scale]
gen laplace [loc] [scale]
gen logistic [loc] [scale] : Logistic distribution - scale > 0
gen pareto [alpha]         : Pareto distribution - alpha > 0
gen rayleigh [scale]       : Rayleigh distribution - scale >= 0
gen uniform [low] [high]   : Uniform distribution - low < high
gen vonmises [mu] [kappa]  : Von Mises distribution - kappa >= 0
gen wald [mean] [scale]    : Wald distribution - mean > 0, scale >= 0
gen weibull [a]            : Weibull distribution - a > 0
gen zipf [a]               : Zipf distribution - a > 1

Publish distribution based on pre-generated file
Possible arguments:
file <file name> [-r]  : Repeat after reaching EOF
"""


import argparse
import os
import sys
import rospy

import generators as GENS
from std_msgs.msg import Float32

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

	_only_zero_choice = "zeroes"
	_huge_error_choice = "huge"


	def __init__(self):
		""" Ctor """

		object.__init__(self)

		self._base_path_expanded = os.path.expanduser(_BASE_PATH)

		generator_choices = GENS.get_generator_names()

		parser = argparse.ArgumentParser(prog="dist_pub")
		parser.add_argument("--id", "-i", required=True, help="ID to publish to")
		intrusion_choices = [self._only_zero_choice, self._huge_error_choice]
		parser.add_argument("--intrusion-mode", "-e", choices=intrusion_choices, dest="intrusion_mode",
			help="One of the possible intrusion modes: {}".format(intrusion_choices))

		# TODO implement intrusions
		raise NotImplementedError()

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

		# Remove remapping arguments and delete program name from arguments
		filtered_argv = rospy.myargv(sys.argv)[1:]

		args = parser.parse_args(filtered_argv)

		return_message = ""
		queue_size = 10

		#    a) File based
		if args.mode == "file":
			return_message = self._setup_reader(args.pub_file_path, args.repeat_file)
		#    b) Generator based
		elif args.mode == "gen":
			return_message = self._setup_generator(args.generator, args.params, args.intrusion_mode)
			queue_size = GENS.GENERATORS[self._sub_routine].queue_size
		else:
			raise NotImplementedError

		# If ID was set we use it as a topic to publish to
		publish_topic = args.id

		rospy.init_node(args.id, anonymous=True)
		rospy.loginfo(return_message)
		self._publisher = rospy.Publisher(publish_topic, Float32, queue_size=queue_size)


	def _setup_reader(self, file_path, repeat_file):
		""" Setup for file-based publishing """

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


	def _setup_generator(self, gen_name, parameters, intrusion_mode):
		""" Setup for data generation """

		try:
			generator = GENS.GENERATORS[gen_name]
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
			rate_in_hz = GENS.GENERATORS[self._sub_routine].rate_in_hz
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
		return GENS.GENERATORS[self._sub_routine].generate()


if __name__ == "__main__":
	try:
		PUB = DistributionPublisher()
		PUB.run()
	except rospy.ROSInterruptException:
		pass
