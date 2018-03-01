#!/usr/bin/env python
""" Module for the PoseProcessor class """

class PoseProcessor(object):
	""" Process poses and convert them to request objects. """

	name = ""


	@staticmethod
	def add_to_request(request, crd_x, crd_y):
		""" Add the given coordinates to the given request and return it. """
		request["x"] = crd_x
		request["y"] = crd_y
		return request


	def __init__(self, name, process):
		""" Ctor """

		object.__init__(self)

		self.name = name
		self.process = process


	# This method is replaced by implementation
	# pylint: disable-msg=R0201,W0613,E0202; (Could be function, unused arguments, method is hidden)
	def process(self, request):
		""" Process the given coordinates and enrich the request with result. """
		raise ValueError("This is not supposed to be called.")


	def process_cc(self, request):
		""" Return the given request unchanged. """
		return request


	def process_poi(self, request):
		pass


	def process_tsp(self, request):
		pass
