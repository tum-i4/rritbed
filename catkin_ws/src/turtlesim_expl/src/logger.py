#!/usr/bin/env python
""" Logging node """

import requests
import rospy
from rosgraph_msgs.msg import Log

URL = "http://localhost:5000"
DATA = """{
	"vin": "A192738"
	}"""


def log(data):
	""" TODO currently logs, should call API """

	requests.post(URL + "/log", data=DATA)


def logger():
	""" Main function """

	rospy.init_node('logger', anonymous=True)

	rospy.Subscriber("/rosout", Log, log)

	# spin() keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	logger()
