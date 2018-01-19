#!/usr/bin/env python
""" Web API """

# pylint: disable-msg=E1101

import time
import datetime
import os.path
from bottle import post, run, request, BaseResponse

from log_entry import LogEntry

LOG_FOLDER = "log"
LOG_FILE_NAME = "log"
LOG_FILE_PATH = os.path.join(LOG_FOLDER, LOG_FILE_NAME)


### API endpoints ###


@post("/log")
def log():
	""" Default log endpoint with no arguments """

	basic_log_entry = LogEntry(vin="", origin="com.status", log_lib_version="5.6.1", appID="STATUS")

	_append_to_log(basic_log_entry.get_log_string())
	return


@post("/log/data/<generator>")
def log_data(generator):
	""" Log endpoint for data generator """
	_log_num(generator, request.params.generated)


def _log_num(name, num):
	""" Log the given number under the given method name """

	method_name = "register" + name.capitalize()

	number_log_entry = LogEntry(
		vin=request.params.vin,
		origin="com.api." + method_name,
		log_lib_version="5.3.2",
		appID=name.upper(),
		log_message="Got {} in {}".format(num, method_name)
	)

	_append_to_log(number_log_entry.get_log_string())


@post("/log/colour")
def log_colour():
	""" Log endpoint with colour input """

	colour_log_entry = LogEntry(
		vin=request.params.vin,
		origin="com.api.web.callColour",
		log_lib_version="5.6.1",
		appID="COLOUR",
		log_message="Successfully registered colour " + request.params.colour
	)

	_append_to_log(colour_log_entry.get_log_string())


@post("/log/get-vins/<num:int>")
def log_num(num):
	""" Log endpoint with number input """

	# pylint: disable-msg=E1101
	numbered_log_entry = LogEntry(
		vin=request.params.vin,
		origin="com.api.web.getVins",
		log_lib_version="5.3.2",
		appID="GETVINS",
		log_message="getVins returned {} vins".format(num))

	_append_to_log(numbered_log_entry.get_log_string())


@post("/DANGER/reset-log")
def reset_log():
	""" Clears the log file """

	if not os.path.isfile(LOG_FILE_PATH):
		return BaseResponse(body="Log file doesn't exist", status=200)

	time_unix = time.time()
	new_file_name = LOG_FILE_PATH + "_until_" + _get_time_string(time_unix)

	while os.path.isfile(new_file_name):
		time_unix += datetime.timedelta(seconds=1)
		new_file_name = LOG_FILE_PATH + _get_time_string(time_unix)

	os.rename(LOG_FILE_PATH, new_file_name)

	return BaseResponse(body="Successfully cleared the log file", status=200)



### Helper methods ###


def _get_time_string(time_unix):
	""" Creates time string of the format '2017-12-20_18:08:25' """

	return time.strftime("%Y-%m-%d_%H:%M:%S", time.gmtime(time_unix))


def _append_to_log(log_str):
	""" Appends the given string plus a newline to the log file """

	with open(LOG_FILE_PATH, "a") as outfile:
		outfile.write(log_str + "\n")
	return


run(host="localhost", port=5000)
