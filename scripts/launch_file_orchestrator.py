#!/usr/bin/env python
"""
Launch file orchestrator
For usage see --help output.
"""

import argparse
import os
import random

import json
from sys import maxint as MAXINT
from lxml import etree as ET

from vin_generator import VinGenerator

GEN_DEFS_FILE_PATH = "~/ros/gens"


class LaunchFileOrchestrator(object):
	""" Creates a launch file based on the given arguments """

	_file_path = ""
	_dump_mode = False

	_manual_turtle_mode = False
	_identifier_file_path = None
	_namespace_count = None

	_intrusion_percentage = 0
	_intrude_turtle = True
	_intrude_generators = True
	_duplicate_vins = False

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		parser = argparse.ArgumentParser(prog="lfo", description="Create a ROS launch file")

		file_or_dump_group = parser.add_mutually_exclusive_group(required=True)
		file_or_dump_group.add_argument("--file", "-f", metavar="/FILE/PATH", dest="file_path",
			help="Write result to specified file")
		file_or_dump_group.add_argument("--dump", "-d", action="store_true", dest="dump_mode",
			help="Dump result to stdout")

		# Optional arguments

		optionals_group = parser.add_argument_group(title="additional options")
		optionals_group.add_argument("--manual", "-m", action="store_true", dest="manual_turtle_mode",
			help=("Create launch file with only one manually controlled turtle, a logger node and "
			"rosbag recording.\nExcludes other options!"))
		optionals_group.add_argument("--identifiers", "-i", dest="identifier_file_path",
			metavar="/FILE/PATH",
			help=("Path to file with identifiers to use for namespaces. Limits number of "
			"namespaces to the number of individual identifiers in file!"))
		optionals_group.add_argument("--namespaces", "-n", type=int, dest="namespace_count",
			metavar="NS_COUNT", help="Number of namespaces to create")

		# Intrusions
		optionals_group.add_argument("--intrusions", "-p", type=int, dest="intrusion_percentage",
			metavar="INTR_PERCENT", default=0, choices=range(1, 101),
			help="Percentage of intrusions to be included in the launch file")
		# Additional intrusion options
		requires_intrusions_text = "[requires --intrusions]"
		optionals_group.add_argument("--dont-intrude-turtle", "-t", action="store_false",
			dest="intrude_turtle",
			help="Set this flag to disallow turtle intrusions " + requires_intrusions_text)
		optionals_group.add_argument("--dont-intrude-generators", "-g", action="store_false",
			dest="intrude_generators",
			help="Set this flag to disallow generator intrusions " + requires_intrusions_text)
		optionals_group.add_argument("--allow-duplicate-vins", "-v", action="store_true",
			dest="duplicate_vins",
			help="Set this flag to allow duplicate VINs " + requires_intrusions_text)

		args = parser.parse_args()

		self._sanity_check_args(args)

		self._generate_launch_file_and_exit(args)


	def _sanity_check_args(self, args):
		""" Makes sure that incompatible options aren't set at the same time """

		if args.manual_turtle_mode:
			args.intrude_turtle = False


	def _generate_launch_file_and_exit(self, args):
		self._dump_mode = args.dump_mode

		# File mode: Sanity check and fix supplied path argument
		if not self._dump_mode:
			path_expanded = os.path.expanduser(args.file_path)
			path = os.path.dirname(path_expanded)
			file_name = os.path.basename(path_expanded)
			launch_ext = ".launch"

			if file_name == "":
				file_name = "ros.launch"
			elif not file_name.endswith(launch_ext):
				file_name += launch_ext

			if not os.path.lexists(path):
				self._print_and_exit("Supplied path {} does not exist.".format(path))

			file_path = os.path.join(path, file_name)
			if os.path.lexists(file_path):
				self._print_and_exit("File {} exists already".format(file_path))

			self._file_path = file_path

		# Make sure that in manual mode no other option has been chosen
		if args.manual_turtle_mode and (
			args.identifier_file_path is not None
			or args.namespace_count is not None):
			self._print_and_exit("When using manual mode, no other arguments are allowed")

		# Set other class variables
		self._manual_turtle_mode = args.manual_turtle_mode
		self._identifier_file_path = args.identifier_file_path
		self._namespace_count = args.namespace_count
		self._intrusion_percentage = args.intrusion_percentage
		self._intrude_turtle = args.intrude_turtle
		self._intrude_generators = args.intrude_generators
		self._duplicate_vins = args.duplicate_vins

		self._create()
		exit()


	def _create(self):
		""" Create the launch file based on the set parameters """
		rand_gen = random.Random()
		root_element = ET.Element("launch")

		# Rosbag recording
		rosbag_folder = os.path.expanduser(os.path.join("~", "ros", "bags", "recording-all"))
		root_element.append(
			self._create_padded_comment("Rosbag recording to the file (prefix) {}".format(rosbag_folder)))
		# <node pkg="rosbag" type="record" name="rosbag_recorder" args="-a -o /file/prefix"/>
		root_element.append(
			self._create_node_element("rosbag_recorder", "record", "rosbag", None, "-a -o " + rosbag_folder))

		vin_list = []

		# Manual mode: Just one namespace group
		if self._manual_turtle_mode:
			vin_list = ["Manual_mode"]

		# Identifier file given: Load identifiers from there
		if self._identifier_file_path is not None:
			vin_list = self._load_identifiers_from_file()
			if self._namespace_count is not None and len(vin_list) < self._namespace_count:
				raise Exception("Given namespace number needs to be leq than given identifiers")

		# Namespace number given: Generate or load appropriate amount of identifiers
		if self._namespace_count is not None:
			if self._identifier_file_path is None:
				vin_list = VinGenerator.generate_vins(self._namespace_count)
			else:
				vin_list = rand_gen.sample(vin_list, self._namespace_count)

		if self._identifier_file_path is None and self._namespace_count is None:
			vin_list = VinGenerator.generate_vins(1)

		print("Creating {} group{}{}".format(
			len(vin_list),
			"s" if len(vin_list) > 1 else "",
			" in manual mode" if self._manual_turtle_mode else ""))

		# Generate a boolean for each VIN denoting if it was intruded
		intrusions = self._generate_intrusions_flags(vin_list, rand_gen)

		for i in range(0, len(vin_list)):
			root_element.append(self._create_unit(vin_list[i], rand_gen, intrusions[i]))

		# Add header comment at top of file
		header_comment = self._create_padded_comment(
			"Launch file with {} namespaces {}".format(
				len(vin_list),
				"(manual mode)" if self._manual_turtle_mode else ""))
		root_element.insert(0, header_comment)

		if self._dump_mode:
			print("") # New line
			ET.dump(root_element)
			exit()

		xml_tree = ET.ElementTree(root_element)

		xml_tree.write(self._file_path, xml_declaration=True)
		print("Successfully saved launch file {}".format(self._file_path))


	def _create_unit(self, vin, rand_gen, intruded=False):
		""" Creates a 'unit' (a 'car') consisting of logging, turtle and data generation """

		group_element = self._create_group([], vin)

		# Turtle group [1]
		# Options:
		# - Manual control
		# - Random walk with parameter input for random seed
		# - Random walk with intelligence

		# Random mover; args: --seed FLOAT --intelligence CHOICE
		# <node name="mover" pkg="turtlesim_expl" type="random_mover.py" args="--seed 1.23" />

		group_element.append(self._create_padded_comment("Turtle group"))

		# TODO: Add more intelligence options
		seed = "{:f}".format(rand_gen.uniform(0, MAXINT))
		intell_choice = rand_gen.choice(["return"])
		if intruded and self._intrude_turtle:
			# TODO: Implement stay logic!
			intell_choice = rand_gen.choice(["stay"])

		control_node_args = "--seed {}{}".format(
			seed,
			" --intelligence " + intell_choice if intell_choice != "" else "")

		control_node = self._create_node_element(
			"mover", "random_mover.py", "turtlesim_expl", n_args=control_node_args)

		if self._manual_turtle_mode:
			control_node = self._create_node_element(
				"teleop", "turtle_teleop_key", "turtlesim")
			control_node.attrib["output"] = "screen"

		group_element.append(
			self._create_turtle_group(control_node))

		if self._manual_turtle_mode:
			return group_element

		# Data generation [1..10]
		# - Based on distributions
		# - A few parameters
		# - Live and file based

		gen_defs_file_path_expanded = os.path.expanduser(GEN_DEFS_FILE_PATH)
		if not os.path.lexists(gen_defs_file_path_expanded):
			raise Exception("Generator definitions file not found at {}".format(gen_defs_file_path_expanded))

		json_line = ""
		with open(gen_defs_file_path_expanded, 'r') as file_reader:
			file_content = file_reader.readlines()
			assert(len(file_content) == 1)
			json_line = file_content[0]

		generator_definitions = json.loads(json_line)

		selected_generators = []
		selected_generator_frequency = {}
		possible_generators = generator_definitions.keys()

		number_of_generators = rand_gen.randint(1, 10)
		for _ in range(0, number_of_generators):
			selection = random.choice(possible_generators)
			selected_generators.append(selection)
			selected_generator_frequency[selection] = 0

		# <node name="gauss" type="distribution_publisher.py" pkg="turtlesim_expl"
		#   args="-i gaussian_1 gen gaussian 1.0 2.0" />
		group_element.append(self._create_padded_comment("Generators"))

		selected_generator_keys = []

		for gen_name in selected_generators:
			selected_generator_frequency[gen_name] += 1
			gen_key = "{}_{}".format(gen_name, selected_generator_frequency[gen_name])
			selected_generator_keys.append(gen_key)

			group_element.append(self._create_generator_node_element(
				gen_key,
				gen_name,
				generator_definitions[gen_name]))

		assert(len(selected_generators) == len(selected_generator_keys))

		# Logging node
		group_element.append(self._create_padded_comment("Logging"))
		# <node ns="log" name="logger" pkg="turtlesim_expl" type="logger.py" args="A1231414" />
		logger_args = "{} --gen-topics".format(vin)
		for gen_key in selected_generator_keys:
			logger_args += " {}".format(gen_key)

		group_element.append(
			self._create_node_element("logger", "logger.py", "turtlesim_expl", n_args=logger_args))

		return group_element


	# pylint: disable-msg=R0201,R0913; (Method could be a function, too many arguments)
	def _create_node_element(self, n_name, n_type, n_pkg, n_ns=None, n_args=None):
		""" Creates an ElementTree element "node" with fixed order attributes """

		node_element = ET.Element("node")

		if n_ns is not None:
			node_element.attrib["ns"] = n_ns

		node_element.attrib["name"] = n_name
		node_element.attrib["type"] = n_type

		if n_args is not None:
			node_element.attrib["args"] = n_args

		node_element.attrib["pkg"] = n_pkg

		return node_element


	def _create_generator_node_element(self, gen_id, gen_key, gen_def):
		""" Creates a generator node element """

		args = "--id {} gen {}".format(gen_id, gen_key)

		for arg_def in gen_def:
			arg = random.uniform(float(arg_def["min"]), float(arg_def["max"]))
			args += " {:f}".format(arg)

		return self._create_node_element(
			gen_id, "distribution_publisher.py", "turtlesim_expl", n_args=args)


	def _create_group(self, elements, n_ns=None):
		""" Creates a group with the given element and optionally the given namespace """

		group_element = ET.Element("group")

		if n_ns is not None:
			group_element.attrib["ns"] = n_ns

		for element in elements:
			group_element.append(element)

		return group_element


	def _create_turtle_group(self, control_node):
		""" Creates a group of ns "turtle" with a turtle and the given control node """

		turtle_node = self._create_node_element("turtlesim", "turtlesim_node", "turtlesim")
		return self._create_group([turtle_node, control_node], n_ns="turtle")


	def _create_padded_comment(self, text):
		""" Creates a comment padded front and back with a space for legibility """

		if "--" in text:
			text.replace("--", "__")

		return ET.Comment(" {} ".format(text.strip()))


	def _generate_intrusions_flags(self, vin_list, rand_gen):
		""" Fills a list of the length of the given list with booleans
		according to the intrusion percentage saved in self """

		# Default: No intrusions
		if self._intrusion_percentage == 0:
			return [False for _ in vin_list]

		# Sample from a ten times bigger list to increase precision
		total_count = len(vin_list) * 10
		intruded_share = int(total_count * (float(self._intrusion_percentage) / 100.0))

		intrusion_choices = (
			[True for _ in range(0, intruded_share)]
			+ [False for _ in range(0, total_count - intruded_share)])

		intrusions = rand_gen.sample(intrusion_choices, len(vin_list))

		assert (len(intrusion_choices) == total_count)
		assert (len(intrusions) == len(vin_list))

		return intrusions


	def _load_identifiers_from_file(self):
		""" Load the identifiers from the file path set in the class """

		if not os.path.lexists(self._identifier_file_path):
			raise Exception("Can't find identifier file at {}".format(self._identifier_file_path))

		identifiers = []
		with open(self._identifier_file_path, "r") as file_reader:
			identifiers = file_reader.readlines()

		return identifiers


	def _print_and_exit(self, text):
		print(text)
		exit()



if __name__ == "__main__":
	LFO = LaunchFileOrchestrator()
