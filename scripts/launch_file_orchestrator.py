#!/usr/bin/env python
"""
Launch file orchestrator

Usage:
</path/to/file> [OPTIONS]

Possible OPTIONS:
-m               : Create launch file with only one manually controlled turtle,
                   a logger node and rosbag recording.
                   Excludes all other options!
-t="TURTLE_TYPE" : Choose turtle type; possible types: RANDOM, RANDOMPI, RANDOMPI1000
-d=NUMBER        : Number of distribution generators; [0,100]
-g=NUMBER        : Number of turtle-based generators; [0,10]
"""

import sys

class LaunchFileOrchestrator(object):
	""" Creates a launch file based on the given arguments """

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		args = sys.argv[1:]

		# TODO: Read arguments


	def create(self):
		pass



if __name__ == "__main__":
	LFO = LaunchFileOrchestrator()
	LFO.create()
