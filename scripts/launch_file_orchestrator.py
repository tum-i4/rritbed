#!/usr/bin/env python
"""
Launch file orchestrator
For usage see _help() method.
"""

import os
import random
import sys

import json
from lxml import etree as ET

GEN_DEFS_FILE_PATH = "~/ros/gens"


class LaunchFileOrchestrator(object):
	""" Creates a launch file based on the given arguments """

	_help_text = """
Usage:
lfo --help : Display this help
lfo --vins <number of vins> [vary plant: true / false]

lfo </path/to/file> [OPTIONS]

Possible OPTIONS:
--manual      : Create launch file with only one manually controlled turtle,
                a logger node and rosbag recording.
                Excludes all other options!
-i="/file"    : Path to file with identifiers to use for namespaces.
                Limits number of namespaces to the number of individual 
                identifiers in file!
-n=NUMBER     : Number of namespaces to create
	"""

	_file_path = ""

	_manual_turtle_mode = False
	_identifier_file_path = None
	_namespace_number = None

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		args = sys.argv[1:]

		if "--help" in args or not args:
			self._print_and_exit(self._help_text)

		# VIN mode
		if "--vins" in args:
			if len(args) < 2 or len(args) > 3:
				self._print_and_exit("Invalid number of arguments supplied")

			number_of_vins = 0
			try:
				number_of_vins = int(args[1])
			except ValueError:
				self._print_and_exit("Invalid value for number of VINs supplied: {}".format(args[1]))

			vary_plant = True
			if len(args) == 3:
				if args[2] == "false":
					vary_plant = False
				else:
					self._print_and_exit("Invalid value for vary plant argument supplied: {}".format(args[2]))

			self._print_vins_and_exit(number_of_vins, vary_plant)

		# Make sure arguments have been supplied
		if not args:
			self._print_and_exit("No arguments supplied")

		# Sanity check supplied path argument
		arg_0_expanded = os.path.expanduser(args[0])
		path = os.path.dirname(arg_0_expanded)
		file_name = os.path.basename(arg_0_expanded)
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

		# Manual mode?
		if "--manual" in args:
			self._manual_turtle_mode = True
			if len(args) > 2:
				self._print_and_exit("When using -m, no other arguments are allowed")
			return

		# Check all remaining arguments
		for arg in args[1:]:
			if arg.startswith("-i="):
				self._identifier_file_path = os.path.expanduser(arg[3:])
			elif arg.startswith("-n="):
				try:
					self._namespace_number = int(arg[3:])
				except ValueError:
					self._print_and_exit(
						"Please supply integer for namespace number.\nReceived: {}".format(arg[3:]))
			else:
				self._print_and_exit("Invalid argument supplied: {}".format(arg))


	def create(self):
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
			if self._namespace_number is not None and len(vin_list) < self._namespace_number:
				raise Exception("Given namespace number needs to be leq than given identifiers")

		# Namespace number given: Generate or load appropriate amount of identifiers
		if self._namespace_number is not None:
			if self._identifier_file_path is None:
				vin_list = self._generate_vins(self._namespace_number)
			else:
				vin_list = rand_gen.sample(vin_list, self._namespace_number)

		if self._identifier_file_path is None and self._namespace_number is None:
			vin_list = self._generate_vins(random.randint(0, 100))

		print("Creating {} groups{}".format(
			len(vin_list), " in manual mode" if self._manual_turtle_mode else ""))

		for vin in vin_list:
			root_element.append(self._create_unit(vin, rand_gen))

		xml_tree = ET.ElementTree(root_element)

		# TODO: Write to file
		xml_tree.write(self._file_path, xml_declaration=True)
		print("Successfully saved launch file {}".format(self._file_path))


	def _create_unit(self, vin, rand_gen):
		""" Creates a 'unit' (a 'car') consisting of logging, turtle and data generation """

		group_element = self._create_group([], vin)

		# Logging node
		group_element.append(self._create_padded_comment("Logging"))
		# <node ns="log" name="logger" pkg="turtlesim_expl" type="logger.py" />
		group_element.append(
			self._create_node_element("logger", "logger.py", "turtlesim_expl", "log"))

		# Turtle group [1]
		# Options:
		# - Manual control
		# - Random walk with parameter input for random seed
		# - Random walk with intelligence

		# Random mover -pi -pi1000 or float for args
		# <node name="mover" pkg="turtlesim_expl" type="random_mover.py" args="-pi1000" />

		group_element.append(self._create_padded_comment("Turtle group"))

		# TODO: Vary parameter input for random seed
		# TODO: Add random walk with intelligence
		control_node = self._create_node_element(
			"mover", "random_mover.py", "turtlesim_expl", n_args="-pi1000")

		if self._manual_turtle_mode:
			control_node = self._create_node_element(
				"teleop", "turtle_teleop_key", "turtlesim")
			control_node.attrib["output"] = "screen"

		group_element.append(
			self._create_turtle_group(control_node))

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
		possible_generators = generator_definitions.keys()
		number_of_generators = rand_gen.randint(1, 10)
		for _ in range(0, number_of_generators):
			selected_generators.append(
				random.choice(possible_generators))

		group_element.append(self._create_padded_comment("Generators"))
		for key in selected_generators:
			group_element.append(self._create_generator_node_element(key, generator_definitions[key]))

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


	def _create_generator_node_element(self, gen_name, gen_def):
		""" Creates a generator node element """

		args = gen_name

		for arg_def in gen_def:
			arg = random.uniform(float(arg_def["min"]), float(arg_def["max"]))
			args += " {:f}".format(arg)

		return self._create_node_element(
			gen_name, "distribution_publisher.py", "turtlesim_expl", n_args=args)

	# 	 <!-- Data generation with gaussian distribution and default arguments -->
	#   <node name="gauss" pkg="turtlesim_expl" type="distribution_publisher.py"
	#     args="gaussian" />


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
		return ET.Comment(" {} ".format(text))


	def _load_identifiers_from_file(self):
		""" Load the identifiers from the file path set in the class """

		if not os.path.lexists(self._identifier_file_path):
			raise Exception("Can't find identifier file at {}".format(self._identifier_file_path))

		identifiers = []
		with open(self._identifier_file_path, "r") as file_reader:
			identifiers = file_reader.readlines()

		return identifiers


	def _generate_vins(self, number_of_vins, vary_plant=True):
		""" Generates VIN tails in the format [A-Z][0-9]{6} (from WBAUV710X0A192738) """

		# Maximum start for one VIN: 999999
		possible_starts = range(100000, 1000001-number_of_vins)
		start = random.choice(possible_starts)

		vins = []
		plant_letter = self._get_plant_letter()

		for serial_number in range(start, start+number_of_vins):
			if vary_plant:
				plant_letter = self._get_plant_letter()
			vins.append(plant_letter + str(serial_number))

		return vins


	def _get_plant_letter(self):
		""" Returns char between A and Z """
		return chr(random.choice(range(65, 91)))


	def _print_vins_and_exit(self, number_of_vins, vary_plant=True):
		""" Generate VINs with the given arguments, print them one by one and exit """

		vins = self._generate_vins(number_of_vins, vary_plant)
		for vin in vins:
			print(vin)

		exit()


	def _print_and_exit(self, text):
		print(text)
		exit()



if __name__ == "__main__":
	LFO = LaunchFileOrchestrator()
	LFO.create()
