#!/usr/bin/env python
""" Logging node """

import argparse
import random
import sys
import time

import requests
import rospy

from turtlesim.msg import Color
from turtlesim.msg import Pose
from std_msgs.msg import Float32


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

	_last_broadcast = {}

	_last_conn_err = 0

	def init(self):
		""" Initialise logger """

		parser = argparse.ArgumentParser(prog="logger")

		parser.add_argument("namespace", metavar="NS", help="The namespace this logger is seated in")
		parser.add_argument("--gen-topics", metavar="TOPIC", nargs="*", default=[], dest="gen_topics")

		args = parser.parse_args(rospy.myargv(sys.argv)[1:])

		self._data[self._vin_field] = args.namespace

		self._last_broadcast[self.log_colour.__name__] = 0
		self._last_broadcast[self.log_pose.__name__] = 0

		rospy.init_node('logger', anonymous=True)

		# Subscribe to topics
		for topic in args.gen_topics:
			rospy.Subscriber(topic, Float32, self.log_generated_data, topic)

		rospy.Subscriber(COLOUR_PATH, Color, self.rate_limit, self.log_colour)
		rospy.Subscriber(POSE_PATH, Pose, self.rate_limit, self.log_pose)


	def log_generated_data(self, data, generator_name):
		""" Logging generated data value """

		request = self._data
		request["generated"] = data.data

		self.send_request("data/" + generator_name, request)


	def rate_limit(self, log_data, method, rate_in_sec=1):
		""" Rate limiting for publishing messages """

		time_now = time.time()

		# Add a degree of randomness to when exactly the logging will occur
		if self._last_broadcast[method.__name__] == 0:
			self._last_broadcast[method.__name__] = time_now + random.randrange(1, 3)

		# Only broadcast once per rate_in_sec
		if time_now < self._last_broadcast[method.__name__] + rate_in_sec:
			return

		self._last_broadcast[method.__name__] = time_now

		method(log_data)


	def log_colour(self, log_data):
		""" Colour logging """

		request = self._data
		request["colour"] = "{},{},{}".format(log_data.r, log_data.g, log_data.b)

		self.send_request("colour", request)


	def log_pose(self, log_data):
		""" Pose logging """

		# 1) Country code request - 50 %
		cco = "country code"
		# 2) POI search - 25 %
		poi = "poi"
		# 3) TSPRouting - 25 %
		tsp = "tsp"

		# Randomly choose which request to make
		choice = random.choice([cco, cco, poi, tsp])

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

		self.send_request("country-code", request, "get")


	def request_random_poi(self, request):
		""" Send request as POI request of random type """
		pass


	def request_random_tsp_routing(self, request):
		# We need: targ_x, targ_y
		# TODO implement
		print("NOT IMPLEMENTED!")


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
