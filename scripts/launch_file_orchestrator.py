#!/usr/bin/env python
"""
Launch file orchestrator
For usage see _help() method.
"""

import sys

class LaunchFileOrchestrator(object):
	""" Creates a launch file based on the given arguments """

	_help_text = """
Usage:
lfo </path/to/file> [OPTIONS]

Possible OPTIONS:
--help     : Display this help
-m         : Create launch file with only one manually controlled turtle,
             a logger node and rosbag recording.
             Excludes all other options!
-i="/file" : Path to file with identifiers to use for namespaces.
             Limits number of namespaces to the number of individual 
             identifiers in file!
-n=NUMBER  : Number of namespaces to create
	"""

	_manual_turtle_mode = False
	_identifier_file_path = ""
	_namespace_number = 0

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		args = sys.argv[1:]

		if "--help" in args or not args:
			self._print_and_exit(self._help_text)

		if "-m" in args:
			self._manual_turtle_mode = True
			if len(args) > 1:
				self._print_and_exit("When using -m, no other arguments are allowed")
			return

		for arg in args:
			if arg.startswith("-i="):
				self._identifier_file_path = arg[3:]
			elif arg.startswith("-n="):
				try:
					self._namespace_number = int(arg[3:])
				except ValueError:
					self._print_and_exit("Please supply integer for namespace number.\nReceived: {}".format(arg[3:]))
			else:
				self._print_and_exit("Invalid argument supplied: {}".format(arg))

		# TODO: Read arguments
		exit()


	def create(self):
		# TODO: Create just one launch file with multiple namespaces!
		# Use VIN as namespace name?
		# "Manual" launch *is* supposed to be one launch file with *just* the manually controlled turtle
		# Pass as arguments instead the number of namespaces or a file with identifiers?
		pass


	def _print_and_exit(self, text):
		print(text)
		exit()



if __name__ == "__main__":
	LFO = LaunchFileOrchestrator()
	LFO.create()
