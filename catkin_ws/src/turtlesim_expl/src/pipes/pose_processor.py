#!/usr/bin/env python
""" Module for the PoseProcessor class """

import random

CC_STR = "cc"
POI_STR = "poi"
TSP_STR = "tsp"


class PoseProcessor(object):
	""" Process poses and convert them to request objects. """

	name = ""

	_x_str = "x"
	_y_str = "y"


	@staticmethod
	def add_to_request(request, crd_x, crd_y):
		""" Add the given coordinates to the given request and return it. """
		request[PoseProcessor._x_str] = crd_x
		request[PoseProcessor._y_str] = crd_y
		return request


	def __init__(self, name):
		""" Ctor """

		object.__init__(self)

		self.name = name
		if name == CC_STR:
			self.process = self.process_cc
		elif name == POI_STR:
			self.process = self.process_poi
		elif name == TSP_STR:
			self.process = self.process_tsp
		else:
			raise ValueError("Invalid process name")


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
		""" Return the given request with random x and y coordinates added to it. """

		targ_x_str = "targ_x"
		targ_y_str = "targ_y"
		self._ensure_keys_not_present(request, targ_x_str, targ_y_str)

		# 1 % chance we request a routing to exactly our own position
		targ_x_choices = [request[self._x_str]] + [random.randrange(0, 500) for _ in range(0, 99)]
		targ_y_choices = [request[self._y_str]] + [random.randrange(0, 500) for _ in range(0, 99)]

		# Target coordinates
		targ_x = random.choice(targ_x_choices)
		targ_y = random.choice(targ_y_choices)

		# Add target coordinates
		request[targ_x_str] = targ_x
		request[targ_y_str] = targ_y

		return request


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
