#!/usr/bin/env python
""" Web API """

import random
import os.path
import uuid
import time
from bottle import post, run, template, request, BaseResponse

LOG_FILE_NAME = "log"


### API endpoints ###


@post("/log")
def log():
	""" Default log endpoint with no arguments """

	append_to_log(get_log_string())
	return


@post("/log/<num:int>")
def log_num(num):
	""" Log endpoint with number input """

	random.seed(num)

	# TODO use random to generate data for log

	append_to_log(get_log_string(method_path="com.api.web.num"))


@post("/log/colour")
def log_colour():
	""" Log endpoint with colour input """

	# pylint: disable-msg=E1101
	colour = request.params.colour
	vin = request.params.vin

	append_to_log(get_log_string(method_path="com.car.colour.getColour(" + colour + ")", vin=vin))


@post("/DANGER/reset-log")
def reset_log():
	""" Clears the log file """

	if not os.path.isfile(LOG_FILE_NAME):
		return BaseResponse(body="Log file doesn't exist", status=200)

	counter = 0
	new_file_name = LOG_FILE_NAME + " (0)"
	while os.path.isfile(new_file_name):
		counter += 1
		new_file_name = LOG_FILE_NAME + " ({})".format(counter)

	os.rename(LOG_FILE_NAME, new_file_name)

	return BaseResponse(body="Successfully cleared the log file", status=200)



### Helper methods ###


def append_to_log(log_str):
	""" Appends the given string to the log file """

	with open(LOG_FILE_NAME, "a") as outfile:
		outfile.write(log_str + "\n")
	return


run(host="localhost", port=5000)
