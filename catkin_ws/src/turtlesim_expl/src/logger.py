#!/usr/bin/env python
""" Logging node """

import requests
import rospy
from turtlesim.msg import Color
from std_msgs.msg import Float32

URL = "http://localhost:5000"
PATH = "/log"

GAUSSIAN = "gaussian"
GUMBEL = "gumbel"
LAPLACE = "laplace"
LOGISTIC = "logistic"
PARETO = "pareto"
RAYLEIGH = "rayleigh"
UNIFORM = "uniform"
VONMISES = "vonmises"
WALD = "wald"
WEIBULL = "weibull"
ZIPF = "zipf"
DATA_GENERATOR_NAMES = [
	GAUSSIAN, GUMBEL, LAPLACE, LOGISTIC, PARETO, RAYLEIGH, UNIFORM, VONMISES, WALD, WEIBULL, ZIPF]

COLOUR_1_PATH = "turtle/turtle1/color_sensor"
COLOUR_2_PATH = "turtle/turtle1/color_sensor"


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

		# Data generation
		for name in DATA_GENERATOR_NAMES:
			rospy.Subscriber("/" + name, Float32, self.log_generated_data, name)

		rospy.Subscriber(COLOUR_1_PATH, Color, self.log_colour, 0)
		rospy.Subscriber(COLOUR_2_PATH, Color, self.log_colour, 1)


	def log_generated_data(self, data, generator_name):
		""" Logging generated data value """

		# TODO: Use ROS master / namespace specific VIN
		request = self._data[0]
		request["generated"] = data.data

		self.send_log_request("data/" + generator_name, request)


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
