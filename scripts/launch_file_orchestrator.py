#!/usr/bin/env python
""" Launch file orchestrator """

import sys

class LaunchFileOrchestrator(object):
	a = 0


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
