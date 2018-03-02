#!/usr/bin/env python
""" Module for the PoseProcessor class """

import random


CC_STR = "cc"
POI_STR = "poi"
TSP_STR = "tsp"


class PoseProcessor(object):
	""" Process poses and convert them to request objects. """

	possible_intrusion_levels = ["easy", "med", "hard"]

	name = ""

	_intrusion = None

	_x_str = "x"
	_y_str = "y"


	@staticmethod
	def add_to_request(request, crd_x, crd_y):
		""" Add the given coordinates to the given request and return it. """
		request[PoseProcessor._x_str] = int(crd_x)
		request[PoseProcessor._y_str] = int(crd_y)
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


	def set_intrusion(self, intrusion=None):
		""" Activate the given intrusion level for this PoseProcessor. """

		if intrusion is not None and intrusion not in self.possible_intrusion_levels:
			raise ValueError("Invalid value for intrusion: {}".format(intrusion))

		self._intrusion = intrusion


	# This method is replaced by implementation
	# pylint: disable-msg=R0201,W0613,E0202; (Could be function, unused arguments, method is hidden)
	def process(self, request):
		""" Process the given coordinates and enrich the request with result. """
		raise ValueError("This is not supposed to be called.")


	def process_cc(self, request):
		""" Return the given request unchanged. """

		if self._intrusion is None:
			return request

		# [Intrusion] Jump to random spot that is at least 10 pixels off
		new_request = self._intrude_cc_request(request)
		# Method chooses likelihood for different intrusion levels
		return self._choose_with_likelihood(request, new_request)


	def _intrude_cc_request(self, request):
		""" Modify the x and y coordinates by at least 10 for a new request.\n
		Does not change the original request object. """

		new_request = dict(request)
		old_x = int(request[self._x_str])
		old_y = int(request[self._y_str])
		new_x = old_x
		new_y = old_y

		while abs(new_x - old_x) < 10:
			new_x = random.randint(0, 499)

		while abs(new_y - old_y) < 10:
			new_y = random.randint(0, 499)

		new_request[self._x_str] = new_x
		new_request[self._y_str] = new_y

		return new_request


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


	def _choose_with_likelihood(self, normal, intruded):
		""" Choose from the two given objects with a likelihood corresponding to the intrusion level. """

		if self._intrusion is None:
			return normal
		# Easy: 50 % likelihood
		elif self._intrusion == self.possible_intrusion_levels[0]:
			return random.choice([normal, intruded])
		# Medium: 30 % likelihood for intruded
		elif self._intrusion == self.possible_intrusion_levels[1]:
			return random.choice([normal] * 70 + [intruded] * 30)
		# Hard: 20 % likelihood for intruded == 1/5
		elif self._intrusion == self.possible_intrusion_levels[2]:
			return random.choice([normal] * 4 + [intruded] * 1)
		else:
			raise NotImplementedError("Not implemented for intrusion value: {}".format(self._intrusion))


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
