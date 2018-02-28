#!/usr/bin/env python
""" Logging node """

import argparse
import random
import sys
import time

import requests
import rospy

from turtlesim.msg import Pose, Color
from turtlesim_expl.msg import GenValue


URL = "http://localhost:5000"

TURTLE_PATH = "turtle/turtle1/"
COLOUR_PATH = TURTLE_PATH + "color_sensor"
POSE_PATH = TURTLE_PATH + "pose"


class Logger(object):
	""" Logger class """

	_vin_field = "vin"
	_data = {
		_vin_field: ""
	}

	_label = False

	_last_broadcast = {}
	_last_conn_err = 0

	_rand_gen = None

	def init(self):
		""" Initialise logger """

		parser = argparse.ArgumentParser(prog="logger")

		parser.add_argument("namespace", metavar="NS", help="The namespace this logger is seated in")
		parser.add_argument("--gen-topics", metavar="TOPIC", nargs="*", default=[], dest="gen_topics")
		parser.add_argument("--label", action="store_true", help="Label the data with intrusion type")

		args = parser.parse_args(rospy.myargv(sys.argv)[1:])

		self._data[self._vin_field] = args.namespace
		self._label = args.label

		self._last_broadcast[self.log_colour.__name__] = 0
		self._last_broadcast[self.log_pose.__name__] = 0

		self._rand_gen = random.Random()

		rospy.init_node("logger", anonymous=True)

		# Subscribe to topics
		for topic in args.gen_topics:
			rospy.Subscriber(topic, GenValue, self.log_generated_data, topic)

		rospy.Subscriber(COLOUR_PATH, Color, self.rate_limit, self.log_colour)
		rospy.Subscriber(POSE_PATH, Pose, self.rate_limit, self.log_pose)


	def log_generated_data(self, gen_value, generator_name):
		""" Log generated data value. """

		request = self._data
		request["generated"] = gen_value.value
		if self._label:
			request["intrusion"] = gen_value.intrusion

		self.send_request("data/" + generator_name, request)


	def rate_limit(self, log_data, method, rate_in_sec=1):
		""" Rate limiting for publishing messages """

		time_now = time.time()

		# Add a degree of randomness to when exactly the logging will occur
		if self._last_broadcast[method.__name__] == 0:
			self._last_broadcast[method.__name__] = time_now + self._rand_gen.randrange(1, 3)

		# Only broadcast once per rate_in_sec
		if time_now < self._last_broadcast[method.__name__] + rate_in_sec:
			return

		self._last_broadcast[method.__name__] = time_now

		method(log_data)


	def log_colour(self, log_data):
		""" Colour logging """

		request = self._data
		request["colour"] = "{},{},{}".format(log_data.r, log_data.g, log_data.b)
		if self._label:
			request["intrusion"] = "normal" if log_data != Color(r=255, g=0, b=0) else "red"

		self.send_request("colour", request)


	def log_pose(self, log_data):
		""" Pose logging - currently can't be labelled for intrusions. """

		# 1) Country code request - 50 %
		cco = "country code"
		# 2) POI search - 25 %
		poi = "poi"
		# 3) TSPRouting - 25 %
		tsp = "tsp"

		# Randomly choose which request to make
		choice = self._rand_gen.choice([cco, cco, poi, tsp])

		request = self._data
		request["x"] = log_data.x
		request["y"] = log_data.y

		if choice == cco:
			self.request_country_code(request)
		elif choice == poi:
			self.request_random_poi(request)
		elif choice == tsp:
			self.request_random_tsp_routing(request)
		else:
			raise NotImplementedError("Choice not implemented")


	def request_country_code(self, request):
		""" Send request as country code request """

		self.send_request("country-code", request, path="get")


	def request_random_poi(self, request):
		""" Send request as POI request of random type """

		# 1) Restaurant - 50 %
		rta = "restaurant"
		# 2) Gas station - 50 %
		gst = "gas station"

		# Add random POI type
		request["type"] = self._rand_gen.choice([rta, gst])

		self.send_request("poi", request, path="get")


	def request_random_tsp_routing(self, request):
		""" Send request as TSP request with random goal """

		# 1 % chance we request a routing to exactly our own position
		targ_x_choices = [request["x"]] + [self._rand_gen.randrange(0, 500) for _ in range(0, 99)]
		targ_y_choices = [request["y"]] + [self._rand_gen.randrange(0, 500) for _ in range(0, 99)]

		# Target coordinates
		targ_x = self._rand_gen.choice(targ_x_choices)
		targ_y = self._rand_gen.choice(targ_y_choices)

		# Add target coordinates
		request["targ_x"] = targ_x
		request["targ_y"] = targ_y

		self.send_request("tsp", request, path="get")


	def send_request(self, log_method, data, path="log"):
		""" Send request to specified logging endpoint with given data """
		try:
			requests.post(URL + "/" + path + "/" + log_method, data)
		except requests.ConnectionError:
			time_now = time.time()
			# Only print an error every second
			if time_now > self._last_conn_err + 1:
				rospy.logerr("Can't connect to logging API")
				self._last_conn_err = time_now



if __name__ == "__main__":
	LOGGER = Logger()
	LOGGER.init()

	# spin() keeps python from exiting until this node is stopped
	rospy.spin()
