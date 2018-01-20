#!/usr/bin/env python
""" Logging node """

import argparse
import sys
import time

import requests
import rospy

from turtlesim.msg import Color
from std_msgs.msg import Float32


URL = "http://localhost:5000"
PATH = "/log"

COLOUR_PATH = "turtle/turtle1/color_sensor"


class Logger(object):
	""" Logger class """

	_vin_field = "vin"

	_data = {
		_vin_field: ""
	}

	_last_colour = None

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


	def log_generated_data(self, data, generator_name):
		""" Logging generated data value """

		request = self._data
		request["generated"] = data.data

		self.send_log_request("data/" + generator_name, request)


	def log_colour(self, log_data):
		""" Colour logging """

		if self._last_colour == log_data:
			return

		self._last_colour = log_data

		request = self._data
		request["colour"] = "{},{},{}".format(log_data.r, log_data.g, log_data.b)

		self.send_log_request("colour", request)


	def send_log_request(self, log_method, data):
		""" Send request to specified logging endpoint with given data """
		try:
			requests.post(URL + PATH + "/" + log_method, data)
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
