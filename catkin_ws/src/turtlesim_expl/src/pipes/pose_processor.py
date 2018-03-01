#!/usr/bin/env python
""" Module for the PoseProcessor class """

import random


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
		""" Return the given request with a random POI type added to it. """

		type_str = "type"
		self._ensure_keys_not_present(request, type_str)

		# Restaurant - 50 %, gas station - 50 %
		request[type_str] = random.choice(["restaurant", "gas station"])
		return request


	def process_tsp(self, request):
		pass


	@staticmethod
	def _ensure_keys_not_present(request, key, *args):
		""" Check the given request object for one or more keys. """

		for key in (key,) + args:
			present = True
			try:
				request[key]
			except KeyError:
				present = False

			if present:
				raise KeyError("Key {} already present in given request".format(key))
