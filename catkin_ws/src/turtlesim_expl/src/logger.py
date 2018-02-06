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

	_last_colour_broadcast = 0
	_last_pose_broadcast = 0

	_last_broadcast = {}

	_last_conn_err = 0

	def init(self):
		""" Initialise logger """

		parser = argparse.ArgumentParser(prog="logger")

		parser.add_argument("namespace", metavar="NS", help="The namespace this logger is seated in")
		parser.add_argument("--gen-topics", metavar="TOPIC", nargs="*", default=[], dest="gen_topics")

		args = parser.parse_args(rospy.myargv(sys.argv)[1:])

		self._data[self._vin_field] = args.namespace

		rospy.init_node('logger', anonymous=True)

		# Subscribe to topics
		for topic in args.gen_topics:
			rospy.Subscriber(topic, Float32, self.log_generated_data, topic)

		rospy.Subscriber(COLOUR_PATH, Color, self.log_colour)
		rospy.Subscriber(POSE_PATH, Pose, self.log_pose)


	def log_generated_data(self, data, generator_name):
		""" Logging generated data value """

		request = self._data
		request["generated"] = data.data

		self.send_request("data/" + generator_name, request)


	def rate_limit(self, log_data, method, rate_in_sec=1):
		""" Rate limiting for publishing messages """

		time_now = time.time()

		# Only broadcast once per rate_in_sec
		if time_now < self._last_broadcast[method.__name__] + rate_in_sec:
			return

		self._last_broadcast[method.__name__] = time_now

		method(log_data)


	def log_colour(self, log_data):
		""" Colour logging """

		# time_now = time.time()

		# # Only broadcast once per second
		# if time_now < self._last_colour_broadcast + 1:
		# 	return

		# self._last_colour_broadcast = time_now

		request = self._data
		request["colour"] = "{},{},{}".format(log_data.r, log_data.g, log_data.b)

		self.send_request("colour", request)


	def log_pose(self, log_data):
		""" Pose logging """

		# Randomly choose which request to make

		# 1) Country code request - 50 %
		cc = "cc"
		# 2) POI search - 25 %
		poi = "poi"
		# 3) TSPRouting - 25 %
		tsp = "tsp"

		choice = random.choice([cc, cc, poi, tsp])

		if choice == cc:
			self.request_country_code(log_data.x, log_data.y)

		# self.send_log_request


	def request_country_code(self, crd_x, crd_y):
		pass


	def request_poi(self, crd_x, crd_y, type="restaurant"):
		pass


	def request_tsp_routing(self, crd_x, crd_y, targ_x, targ_y):
		pass


	def send_request(self, log_method, data, path="/log"):
		""" Send request to specified logging endpoint with given data """
		try:
			requests.post(URL + path + "/" + log_method, data)
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
