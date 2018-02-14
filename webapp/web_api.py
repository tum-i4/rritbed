#!/usr/bin/env python
""" Web API """

# pylint: disable-msg=E1101

import argparse
import datetime
import io
import json
import os.path
import random
import shutil
import time
from bottle import post, run, request, BaseResponse

from log_entry import LogEntry
from state_dao import StateDao
from functionality.country_code_mapper import CountryCodeMapper
from functionality.poi_mapper import PoiMapper
from functionality.tsp_routing_mapper import TspRoutingMapper

LOG_FOLDER = "log"
LOG_FILE_NAME = "log"
LOG_FILE_PATH = os.path.join(LOG_FOLDER, LOG_FILE_NAME)
STATE_FILE_PATH = "state"

LOG_FILE_HANDLE = None


### API endpoints ###


@post("/log")
def log():
	""" Default log endpoint with no arguments """

	basic_log_entry = _create_base_log_entry(request.params.vin)

	basic_log_entry.complete(
		origin="com.status", log_lib_version="5.6.1", app_id="STATUS")

	_append_to_log(basic_log_entry)
	return


@post("/log/data/<generator>")
def log_data(generator):
	""" Log endpoint for data generator """
	_log_num(generator, request.params.generated)


def _log_num(name, num):
	""" Log the given number under the given method name """

	number_log_entry = _create_base_log_entry(request.params.vin)

	method_name = "register" + name.capitalize()

	number_log_entry.complete(
		origin="com.api." + method_name,
		log_lib_version="5.3.2",
		app_id=name.upper(),
		log_message=num
	)

	_append_to_log(number_log_entry)


@post("/get/country-code")
def get_country_code():
	""" Map coordinates to country code and save request and response to log """

	cc_request_log_entry = _create_base_log_entry(request.params.vin)

	crd_x = request.params.x
	crd_y = request.params.y

	origin = "com.get.countryCode"
	lib_version = "6.4.1"
	app_id = "COUNTRYCODE"
	position = _get_position_string(crd_x, crd_y)

	# Save request to log
	cc_request_log_entry.complete(
		origin=origin,
		log_lib_version=lib_version,
		app_id=app_id,
		log_message="Requesting country code",
		gps_position=position
	)

	_append_to_log(cc_request_log_entry)

	country_code = CountryCodeMapper.map(crd_x, crd_y)

	cc_response_log_entry = _create_base_log_entry(request.params.vin)

	# Save response to log
	cc_response_log_entry.complete(
		origin=origin,
		log_lib_version=lib_version,
		app_id=app_id,
		log_message="Country code response [{}] returned for request [x: {} and y: {}]".format(
			country_code, crd_x, crd_y),
		gps_position=position
	)

	_append_to_log(cc_response_log_entry)


@post("/get/poi")
def get_poi():
	""" Map coordinates to POI of given type and save request and response to log """

	poi_request_log_entry = _create_base_log_entry(request.params.vin)

	crd_x = request.params.x
	crd_y = request.params.y
	poi_type = request.params.type

	origin = "com.get.poi"
	lib_version = "6.4.2"
	app_id = "POI"
	position = _get_position_string(crd_x, crd_y)

	# Save request to log
	poi_request_log_entry.complete(
		origin=origin,
		log_lib_version=lib_version,
		app_id=app_id,
		log_message="Requesting POI of type {}".format(poi_type),
		gps_position=position
	)

	_append_to_log(poi_request_log_entry)

	poi_result = PoiMapper.map(poi_type, crd_x, crd_y)

	poi_response_log_entry = _create_base_log_entry(request.params.vin)
	log_message = "Invalid POI type {}!".format(poi_type)
	level = LogEntry.LEVEL_ERROR

	if poi_result is not None:
		log_message = "POI response [{}] returned for request [x: {}, y: {}, type: {}]".format(
			poi_result, crd_x, crd_y, poi_type)
		level = LogEntry.LEVEL_DEFAULT

	# Save response to log
	poi_response_log_entry.complete(
		origin=origin,
		log_lib_version=lib_version,
		app_id=app_id,
		log_message=log_message,
		gps_position=position,
		level=level
	)

	_append_to_log(poi_response_log_entry)


@post("/get/tsp")
def get_tsp_routing():
	""" Map current and goal coordinates to TSP and save request and response to log """

	tsp_request_log_entry = _create_base_log_entry(request.params.vin)

	crd_x = request.params.x
	crd_y = request.params.y
	targ_x = request.params.targ_x
	targ_y = request.params.targ_y

	origin = "com.get.tspRouting"
	lib_version = "6.4.5"
	app_id = "TSPROUTING"
	position = _get_position_string(crd_x, crd_y)

	# Save request to log
	tsp_request_log_entry.complete(
		origin=origin,
		log_lib_version=lib_version,
		app_id=app_id,
		log_message="Requesting TSP routing to target [x: {}, y: {}]".format(
			targ_x, targ_y),
		gps_position=position
	)

	_append_to_log(tsp_request_log_entry)

	tsp_message = TspRoutingMapper.map(crd_x, crd_y, targ_x, targ_y)

	tsp_response_log_entry = _create_base_log_entry(request.params.vin)

	if tsp_message == TspRoutingMapper.GOAL_REACHED_MSG:
		tsp_message = "Goal already reached at {}/{}".format(crd_x, crd_y)

	# Save request to log
	tsp_response_log_entry.complete(
		origin=origin,
		log_lib_version=lib_version,
		app_id=app_id,
		log_message=("TSP routing response [{}] returned for request ".format(tsp_message) +
			"[x: {}, y: {}, target_x: {}, target_y: {}]".format(crd_x, crd_y, targ_x, targ_y)),
		gps_position=position
	)

	_append_to_log(tsp_response_log_entry)


@post("/log/colour")
def log_colour():
	""" Log endpoint with colour input """

	colour_log_entry = _create_base_log_entry(request.params.vin)

	colour_log_entry.complete(
		origin="com.api.web.callColour",
		log_lib_version="5.6.1",
		app_id="COLOUR",
		log_message=request.params.colour
	)

	_append_to_log(colour_log_entry)


@post("/log/get-vins/<num:int>")
def log_num(num):
	""" Log endpoint with number input """

	numbered_log_entry = _create_base_log_entry(request.params.vin)

	numbered_log_entry.complete(
		origin="com.api.web.getVins",
		log_lib_version="5.3.2",
		app_id="GETVINS",
		log_message="getVins returned {} vins".format(num))

	_append_to_log(numbered_log_entry)



### DANGER zone


@post("/DANGER/cut-log")
def cut_log():
	""" Cuts the log off at the common minimum time of all clients """

	minimum_time = StateDao.get_current_min_time()

	if not os.path.isfile(LOG_FILE_PATH):
		return BaseResponse(body="Log is empty", status=200)

	print("Minimum time is {}. Now reading the whole log file - this might take some time...".format(
		time.strftime("%Y-%m-%d, %H:%M", time.gmtime(minimum_time))))

	new_file_path = _create_unique_log_file_path()

	shutil.copyfile(LOG_FILE_PATH, new_file_path)

	log_lines = []
	with open(new_file_path, "r") as new_log_file:
		log_lines = new_log_file.readlines()

	log_length = len(log_lines)

	print("Processing...")

	current_index = len(log_lines) - 1
	while True:
		entry = json.loads(log_lines[current_index])
		if entry[LogEntry.time_unix_field] <= minimum_time:
			break
		log_lines.pop()
		current_index -= 1

	print("Writing results back to disk as {}...".format(new_file_path))

	with open(new_file_path, "w") as outfile:
		outfile.writelines(log_lines)

	message = "Process finished! Removed {} from the original {} lines.\nSaved file to: {}".format(
		log_length - len(log_lines), log_length, new_file_path)
	print(message)

	return BaseResponse(body=message, status=200)


@post("/DANGER/reset-log")
def reset_log():
	""" Clears the log file """

	if not os.path.isfile(LOG_FILE_PATH):
		return BaseResponse(body="Log file doesn't exist", status=200)

	new_file_name = _create_unique_log_file_path()
	os.rename(LOG_FILE_PATH, new_file_name)

	return BaseResponse(body="Successfully cleared the log file", status=200)



### Helper methods ###


def _get_position_string(crd_x, crd_y):
	""" Creates a position string of the format '41.123,40.31312' """
	return "{},{}".format(crd_x, crd_y)


def _create_base_log_entry(vin):
	""" Verifies the given VIN and creates a log entry with the current client time. """

	if vin is None:
		raise ValueError("No VIN given!")

	time_unix = _create_client_time(vin)
	return LogEntry.create_base_entry(vin, time_unix)


def _create_client_time(identifier):
	""" Creates a time for the client. Randomly increments time with 5 % chance. """

	client_time = _get_client_time(identifier)
	time_now = time.time()

	if client_time is None:
		_set_client_time(identifier, time_now)
		return time_now

	time_choice = random.choice([client_time] * 19 + [client_time + random.randint(3600, 57600)])
	_set_client_time(identifier, time_choice)

	return time_choice


def _get_client_time(identifier):
	""" Getter for the client time. Initialises empty field. """

	state = _get_state()
	try:
		return state[CCT_KEY][identifier]
	except KeyError:
		_set_client_time(identifier, None)
		return None


def _create_unique_log_file_path():
	""" Creates a unique log file name for backups """

	time_unix = time.time()
	new_file_name = _create_log_file_name_from_time(LOG_FILE_PATH, time_unix)

	while os.path.isfile(new_file_name):
		time_unix += datetime.timedelta(seconds=1)
		new_file_name = _create_log_file_name_from_time(LOG_FILE_PATH, time_unix)

	return new_file_name


def _create_log_file_name_from_time(path, time_unix):
	""" Creates a log file name of the format 'log/log_until_2017-12-20_18:08:25' """
	return path + "_until_" + time.strftime("%Y-%m-%d_%H:%M:%S", time.gmtime(time_unix))



### Writing to log


def _append_to_log(new_log_entry):
	""" Appends the given string plus a newline to the log file """

	LOG_FILE_HANDLE.write(new_log_entry.get_log_string() + "\n")
	return



### Starting the server


PARSER = argparse.ArgumentParser()
PARSER.add_argument("--quiet", "-q", action="store_true")
ARGS = PARSER.parse_args()

if ARGS.quiet:
	print("Starting server in quiet mode")

# Buffer size increased for improved performance
LOG_FILE_HANDLE = open(LOG_FILE_PATH, "a", io.DEFAULT_BUFFER_SIZE * 10000)

run(host="localhost", port=5000, quiet=ARGS.quiet)

LOG_FILE_HANDLE.close()
