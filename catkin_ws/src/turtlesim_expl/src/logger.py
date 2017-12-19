#!/usr/bin/env python
""" Logging node """

import requests
import rospy
from turtlesim.msg import Color

URL = "http://localhost:5000"

DATA_1 = """{
	"vin": "A192738"
	}"""
DATA_2 = """{
	"vin": "A232758"
	}"""

COLOUR_1_PATH = "/ecu1/turtle1/color_sensor"
COLOUR_2_PATH = "/ecu2/turtle1/color_sensor"


def log_colour_1(data):
	""" Colour logging for node 1 """
	log_colour(data, DATA_1)


def log_colour_2(data):
	""" Colour logging for node 2 """
	log_colour(data, DATA_2)


def log_colour(log_data, request):
	""" Colour logging """

	rospy.loginfo(log_data)

	#requests.post(URL + "/log", data=request)


def logger():
	""" Main function """

	rospy.init_node('logger', anonymous=True)

	rospy.Subscriber(COLOUR_1_PATH, Color, log_colour_1)
	rospy.Subscriber(COLOUR_2_PATH, Color, log_colour_2)

	# spin() keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	logger()
