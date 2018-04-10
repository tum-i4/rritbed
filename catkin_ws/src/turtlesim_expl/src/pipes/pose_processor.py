#!/usr/bin/env python
""" Module for the PoseProcessor class """

import random


CC_STR = "cc"
POI_STR = "poi"
TSP_STR = "tsp"


class PoseProcessor(object):
	""" Process poses and convert them to request objects. """

	POSSIBLE_INTRUSION_LEVELS = ["easy", "med", "hard"]

	_NOT_INTRUDED_LABEL = "normal"
	_JUMP_LABEL = "jump"
	_ILLEGAL_TYPE_LABEL = "illegaltype"
	_ROUTE_TO_SELF_LABEL = "routetoself"

	_X_STR = "x"
	_Y_STR = "y"


	@staticmethod
	def add_to_request(request, crd_x, crd_y):
		""" Add the given coordinates to the given request and return it. """
		request[PoseProcessor._X_STR] = int(crd_x)
		request[PoseProcessor._Y_STR] = int(crd_y)
		return request


	def __init__(self, name):
		""" Ctor """

		object.__init__(self)

		self.name = name
		processor_choices = {
			CC_STR : self.process_cc,
			POI_STR : self.process_poi,
			TSP_STR : self.process_tsp
		}
		# Raises for invalid names
		self.process = processor_choices[name]

		self._intrusion = None
		self._intrusion_field = None


	def set_intrusion(self, intrusion=None, intrusion_field=None):
		""" Activate the given intrusion level for this PoseProcessor. """

		if intrusion is not None:
			if intrusion not in self.POSSIBLE_INTRUSION_LEVELS:
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

		# [Intrusion] Jump to random spot that is at least 10 pixels off
		new_request = self._intrude_cc_request(request)
		# Method chooses likelihood for different intrusion levels
		intrude = self._is_intruded()

		if intrude:
			request = new_request

		if label:
			self._label_request(request, intruded=intrude, intrusion_label=PoseProcessor._JUMP_LABEL)

		return request


	def _intrude_cc_request(self, request):
		""" Modify the x and y coordinates by at least 10 for a new request.\n
		Does not change the original request object. """

		new_request = dict(request)
		old_x = int(request[self._X_STR])
		old_y = int(request[self._Y_STR])
		new_x = old_x
		new_y = old_y

		while abs(new_x - old_x) < 10:
			new_x = random.randint(0, 499)

		while abs(new_y - old_y) < 10:
			new_y = random.randint(0, 499)

		new_request[self._X_STR] = new_x
		new_request[self._Y_STR] = new_y

		return new_request


	def process_poi(self, request, label):
		""" Return the given request with a random POI type added to it. """

		type_str = "type"
		self._ensure_keys_not_present(request, type_str)

		# Restaurant - 50 %, gas station - 50 %
		choice = random.choice(["restaurant", "gas station"])
		# [Intrusion] Illegal type
		intruded_choice = random.choice(["private home", "nsa hq"])

		intrude = self._is_intruded()

		if intrude:
			choice = intruded_choice

		request[type_str] = choice

		if label:
			self._label_request(request, intruded=intrude, intrusion_label=PoseProcessor._ILLEGAL_TYPE_LABEL)

		return request


	def process_tsp(self, request, label):
		""" Return the given request with random x and y coordinates added to it. """

		targ_x_str = "targ_x"
		targ_y_str = "targ_y"
		self._ensure_keys_not_present(request, targ_x_str, targ_y_str)

		targ_x_y = (random.randint(0, 499), random.randint(0, 499))
		# [Intrusion] Request a routing to exactly our own position
		intruded_x_y = (request[self._X_STR], request[self._Y_STR])

		intrude = self._is_intruded()

		if intrude:
			targ_x_y = intruded_x_y

		if label:
			self._label_request(request, intruded=intrude,
				intrusion_label=PoseProcessor._ROUTE_TO_SELF_LABEL)

		targ_x = targ_x_y[0]
		targ_y = targ_x_y[1]

		# Add target coordinates
		request[targ_x_str] = targ_x
		request[targ_y_str] = targ_y

		return request


	def _is_intruded(self):
		""" Choose from the two given objects with a likelihood corresponding to the intrusion level. """
		return self._intrusion is not None


	def _label_request(self, request, intruded, intrusion_label):
		""" Label request in self._intrusion_field with the required label. """

		if not intruded:
			request[self._intrusion_field] = self._NOT_INTRUDED_LABEL
		else:
			request[self._intrusion_field] = intrusion_label


	@staticmethod
	def _ensure_keys_not_present(request, key, *args):
		""" Check the given request object for one or more keys. """

		for k in (key,) + args:
			present = True
			try:
				request[k]
			except KeyError:
				present = False

			if present:
				raise KeyError("Key {} already present in given request".format(k))
