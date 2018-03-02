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

# pylint: disable-msg=C0411; (Standard import should be above - I don't consider it "standard")
from pipes.pose_pipe import PosePipe
from pipes.pose_processor import PoseProcessor, CC_STR, POI_STR, TSP_STR


URL = "http://localhost:5000"

TURTLE_PATH = "turtle/turtle1/"
COLOUR_PATH = TURTLE_PATH + "color_sensor"
POSE_PATH = TURTLE_PATH + "pose"


class Logger(object):
	""" Logger class """

	_vin_field = "vin"
	_intrusion_field = "intrusion"
	_data = {
		_vin_field: ""
	}

	_label = False
	_intrusion = None

	_last_broadcast = {}
	_last_conn_err = 0

	_rand_gen = None

	def init(self):
		""" Ctor """

		parser = argparse.ArgumentParser(prog="logger")

		parser.add_argument("namespace", metavar="NS", help="The namespace this logger is seated in")
		parser.add_argument("--gen-topics", metavar="TOPIC", nargs="*", default=[], dest="gen_topics")
		parser.add_argument("--label", action="store_true", help="Label the data with intrusion type")
		parser.add_argument("--intrusion", "-i", choices=PoseProcessor.possible_intrusion_levels)

		args = parser.parse_args(rospy.myargv(sys.argv)[1:])

		self._data[self._vin_field] = args.namespace
		self._label = args.label
		self._intrusion = args.intrusion

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

		request = self.copy_base_request()
		request["generated"] = gen_value.value
		if self._label:
			request[self._intrusion_field] = gen_value.intrusion

		self.send_request("data/" + generator_name, request)


	def rate_limit(self, log_data, method, rate_in_sec=1):
		""" Rate limiting for publishing messages """

		time_now = time.time()

		# Add a degree of randomness to when exactly the logging will begin
		if self._last_broadcast[method.__name__] == 0:
			self._last_broadcast[method.__name__] = time_now + self._rand_gen.randrange(1, 3)

		# Only broadcast once per rate_in_sec
		if time_now < self._last_broadcast[method.__name__] + rate_in_sec:
			return

		self._last_broadcast[method.__name__] = time_now

		method(log_data)


	def log_colour(self, log_data):
		""" Colour logging """

		request = self.copy_base_request()
		request["colour"] = "{},{},{}".format(log_data.r, log_data.g, log_data.b)
		if self._label:
			request[self._intrusion_field] = "normal" if log_data != Color(r=255, g=0, b=0) else "red"

		self.send_request("colour", request)


	def log_pose(self, log_data):
		""" Pose logging - currently can't be labelled for intrusions. """

		# Country code request - 50 %, POI search / TSP routing - 25 %
		pose_pipe = PosePipe.create(
			intrusion=self._intrusion, intrusion_field=self._intrusion_field,
			cc=50, poi=25, tsp=25)

		request = self.copy_base_request()
		request = PoseProcessor.add_to_request(request, log_data.x, log_data.y)
		request = pose_pipe.process(request, label=self._label)

		processor_name = pose_pipe.get_processor_name()
		endpoint = ""
		if processor_name == CC_STR:
			endpoint = "country-code"
		elif processor_name == POI_STR:
			endpoint = "poi"
		elif processor_name == TSP_STR:
			endpoint = "tsp"
		else:
			raise NotImplementedError("Choice not implemented")

		if self._label:
			request[self._intrusion_field] = "MISSING LABELS ERROR"

		self.send_request(endpoint, request, path="get")


	def send_request(self, log_method, request, path="log"):
		""" Send request to specified logging endpoint with given data. """

		if self._label:
			# Provoke KeyError to ensure existence of "intrusion" key
			# pylint: disable-msg=W0104; (Statement has no effect - does raise)
			request[self._intrusion_field]

		try:
			requests.post(URL + "/" + path + "/" + log_method, request)
		except requests.ConnectionError:
			time_now = time.time()
			# Only print an error every second
			if time_now > self._last_conn_err + 1:
				rospy.logerr("Can't connect to logging API")
				self._last_conn_err = time_now


	def copy_base_request(self):
		""" Make a value-copy of the base request. """
		return dict(self._data)



if __name__ == "__main__":
	LOGGER = Logger()
	LOGGER.init()

	# spin() keeps python from exiting until this node is stopped
	rospy.spin()
