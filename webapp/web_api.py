#!/usr/bin/env python
""" Web API """

import random
import os.path
from bottle import post, run, template, request, BaseResponse

from log_entry import LogEntry

LOG_FILE_NAME = "log"


### API endpoints ###


@post("/log")
def log():
	""" Default log endpoint with no arguments """

	basic_log_entry = LogEntry(vin="", origin="com.status", appID="STATUS")

	append_to_log(basic_log_entry.get_log_string())
	return


@post("/log/get-vins/<num:int>")
def log_num(num):
	""" Log endpoint with number input """

	# pylint: disable-msg=E1101
	numbered_log_entry = LogEntry(
		vin=request.params.vin,
		origin="com.api.web.getVins",
		appID="GETVINS",
		log_message="getVins returned {} vins".format(num))

	append_to_log(numbered_log_entry.get_log_string())


@post("/log/colour")
def log_colour():
	""" Log endpoint with colour input """

	# pylint: disable-msg=E1101
	colour_log_entry = LogEntry(
		vin=request.params.vin,
		origin="com.api.web.callColour",
		appID="COLOUR",
		log_message="Successfully registered colour " + request.params.colour
	)

	append_to_log(colour_log_entry.get_log_string())


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
	""" Appends the given string plus a newline to the log file """

	with open(LOG_FILE_NAME, "a") as outfile:
		outfile.write(log_str + "\n")
	return


run(host="localhost", port=5000)
