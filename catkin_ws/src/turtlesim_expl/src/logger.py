#!/usr/bin/env python
""" Logging node """

import requests
import rospy
from turtlesim.msg import Color
from std_msgs.msg import Float32

URL = "http://localhost:5000"
PATH = "/log"

GAUSSIAN_PATH = "/gaussian_generator"
COLOUR_1_PATH = "/ecu1/turtle1/color_sensor"
COLOUR_2_PATH = "/ecu2/turtle1/color_sensor"


class Logger(object):
	""" Logger class """

	_vin_field = "vin"

	_data = [
		{_vin_field: "A192738"},
		{_vin_field: "A232758"}
	]

	_last_colour = [
		None,
		None
	]

	def init(self):
		""" Initialise logger """

		rospy.init_node('logger', anonymous=True)

		rospy.Subscriber(GAUSSIAN_PATH, Float32, self.log_gaussian)
		rospy.Subscriber(COLOUR_1_PATH, Color, self.log_colour_1)
		rospy.Subscriber(COLOUR_2_PATH, Color, self.log_colour_2)


	def log_gaussian(self, data):
		""" Logging Gaussian value """

		# TODO: Use ROS master specific VIN
		request = self._data[0]
		request["gauss_value"] = data.data

		self.send_log_request("gauss", request)


	def log_colour_1(self, data):
		""" Colour logging for node 1 """
		self.log_colour(data, 0)


	def log_colour_2(self, data):
		""" Colour logging for node 2 """
		self.log_colour(data, 1)


	def log_colour(self, log_data, index):
		""" Colour logging """

		if self._last_colour[index] == log_data:
			return

		self._last_colour[index] = log_data

		request = self._data[index]
		request["colour"] = "{},{},{}".format(log_data.r, log_data.g, log_data.b)

		self.send_log_request("colour", request)


	def send_log_request(self, log_method, data):
		""" Send request to specified logging endpoint with given data """
		requests.post(URL + PATH + "/" + log_method, data)



if __name__ == "__main__":
	LOGGER = Logger()
	LOGGER.init()

	# spin() keeps python from exiting until this node is stopped
	rospy.spin()
