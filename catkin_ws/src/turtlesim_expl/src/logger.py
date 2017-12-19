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

#COLOUR_1_PATH = "/ecu1/turtle1/color_sensor"
#COLOUR_2_PATH = "/ecu2/turtle1/color_sensor"



class Logger:

	last_colour = None

	def init(self):
		""" Initialise logger """

		rospy.init_node('logger', anonymous=True)

		#rospy.Subscriber(COLOUR_1_PATH, Color, self.log_colour_1)
		#rospy.Subscriber(COLOUR_2_PATH, Color, self.log_colour_2)

		rospy.Subscriber("/turtle1/color_sensor", Color, self.log_colour_1)
	

	def log_colour_1(self, data):
		""" Colour logging for node 1 """
		self.log_colour(data, DATA_1)


	def log_colour_2(self, data):
		""" Colour logging for node 2 """
		self.log_colour(data, DATA_2)


	def log_colour(self, log_data, request):
		""" Colour logging """

		if self.last_colour == log_data:
			return

		self.last_colour = log_data
		rospy.loginfo(log_data)

		#requests.post(URL + "/log", data=request)



if __name__ == "__main__":
	LOGGER = Logger()
	LOGGER.init()

	# spin() keeps python from exiting until this node is stopped
	rospy.spin()
