#!/usr/bin/env python
"""
Launch file orchestrator
For usage see _help() method.
"""

import os
import random
import sys

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
	_identifier_file_path = ""
	_namespace_number = 0

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
		path = os.path.dirname(args[0])
		file_name = os.path.basename(args[0])
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
				self._identifier_file_path = arg[3:]
			elif arg.startswith("-n="):
				try:
					self._namespace_number = int(arg[3:])
				except ValueError:
					self._print_and_exit(
						"Please supply integer for namespace number.\nReceived: {}".format(arg[3:]))
			else:
				self._print_and_exit("Invalid argument supplied: {}".format(arg))


	def create(self):
		# TODO: Create just one launch file with multiple namespaces!
		# Use VIN as namespace name
		# "Manual" launch *is* supposed to be one launch file with *just* the manually controlled turtle
		# Based on identifier file
		# Based on number of instances
		# Check if identifier file length and number of instances supplied fit
		pass


#   <!-- Logging -->
#   <node ns="log" name="logger" pkg="turtlesim_expl" type="logger.py" />

#   <!-- Rosbag recording with prefix as given -->
#   <node pkg="rosbag" type="record" name="rosbag_recorder"
#     args=""/>


	def _create_node_element(self, n_name, n_type, n_pkg, n_ns=None, n_args=None):
		""" Creates an ElementTree element "node" with fixed order attributes """

		node_element = ET.Element("node")

		if n_ns is not None:
			node_element.attrib["ns"] = n_ns

		node_element.attrib["name"] = n_name
		node_element.attrib["type"] = n_type
		node_element.attrib["pkg"] = n_pkg

		if n_args is not None:
			node_element.attrib["args"] = n_args

		return node_element


	def _create_group(self, elements, ns=None):
		group_element = ET.Element("group")

		if ns is not None:
			group_element.attrib["ns"] = ns

		for element in elements:
			group_element.append(element)

		return group_element


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
