#!/usr/bin/env python
""" Module for the PoseProcessor class """

import random


CC_STR = "cc"
POI_STR = "poi"
TSP_STR = "tsp"


class PoseProcessor(object):
	""" Process poses and convert them to request objects. """

	possible_intrusion_levels = ["easy", "med", "hard"]
	_not_intruded_label = "normal"

	name = ""

	_intrusion = None
	_intrusion_field = None

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


	def set_intrusion(self, intrusion=None, intrusion_field=None):
		""" Activate the given intrusion level for this PoseProcessor. """

		if intrusion is not None:
			if intrusion not in self.possible_intrusion_levels:
				raise ValueError("Invalid value for intrusion: {}".format(intrusion))
			if intrusion_field is None:
				raise ValueError("If intrusion is set, an intrusion field is required")

		self._intrusion = intrusion
		self._intrusion_field = intrusion_field


	# This method is replaced by implementation
	# pylint: disable-msg=R0201,W0613,E0202; (Could be function, unused arguments, method is hidden)
	def process(self, request, label):
		""" Process the given coordinates and enrich the request with result. """
		raise ValueError("This is not supposed to be called.")


	def process_cc(self, request, label):
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


	def process_poi(self, request, label):
		""" Return the given request with a random POI type added to it. """

		type_str = "type"
		self._ensure_keys_not_present(request, type_str)

		# Restaurant - 50 %, gas station - 50 %
		legal_choice = random.choice(["restaurant", "gas station"])
		# [Intrusion] Illegal type
		intruded_choice = random.choice(["private home", "nsa hq"])

		request[type_str] = self._choose_with_likelihood(legal_choice, intruded_choice)

		return request


	def process_tsp(self, request, label):
		""" Return the given request with random x and y coordinates added to it. """

		targ_x_str = "targ_x"
		targ_y_str = "targ_y"
		self._ensure_keys_not_present(request, targ_x_str, targ_y_str)

		targ_x_y = (random.randint(0, 499), random.randint(0, 499))
		# [Intrusion] Request a routing to exactly our own position
		intruded_x_y = (request[self._x_str], request[self._y_str])

		choice = self._choose_with_likelihood(targ_x_y, intruded_x_y)
		targ_x = choice[0]
		targ_y = choice[1]

		# Add target coordinates
		request[targ_x_str] = targ_x
		request[targ_y_str] = targ_y

		return request


	def _is_intruded_with_likelihood(self):
		""" Choose from the two given objects with a likelihood corresponding to the intrusion level. """

		normal = False
		intruded = True

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


	def _label_request(self, request, intruded=False, intrusion_label=None):
		""" Label request in self._intrusion_field with the required label. """

		if intruded and intrusion_label is None:
			raise ValueError("If intruded, an intrusion label must be given")

		if not intruded:
			request[self._intrusion_field] = self._not_intruded_label
		else:
			request[self._intrusion_field] = intrusion_label


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
