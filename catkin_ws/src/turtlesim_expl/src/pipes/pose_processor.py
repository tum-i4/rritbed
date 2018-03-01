#!/usr/bin/env python
""" Module for the PoseProcessor class """

class PoseProcessor(object):
	""" Process poses and convert them to request objects. """

	_name = ""

	def __init__(self, name, process):
		""" Ctor """

		object.__init__(self)

		self._name = name
		self.process = process


	# This method is replaced by implementation
	# pylint: disable-msg=R0201,W0613,E0202; (Could be function, unused arguments, method is hidden)
	def process(self, crd_x, crd_y, request):
		""" Process the given coordinates and enrich the request with result. """
		raise ValueError("This is not supposed to be called.")


	def process_cc(self, crd_x, crd_y, request):
		pass


	def process_poi(self, crd_x, crd_y, request):
		pass


	def process_tsp(self, crd_x, crd_y, request):
		pass
