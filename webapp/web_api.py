#!/usr/bin/env python
""" Web API """

# pylint: disable-msg=E1101

import argparse
import io
import json
import os.path
import random
import time
from bottle import post, run, request, BaseResponse

from log_entry import LogEntry
from state_dao import StateDao
from functionality.country_code_mapper import CountryCodeMapper
from functionality.poi_mapper import PoiMapper
from functionality.tsp_routing_mapper import TspRoutingMapper


### API endpoints ###


@post("/log")
def log():
	""" Default log endpoint with no arguments """

	basic_log_entry = _create_base_log_entry(request.params.vin)

	basic_log_entry.complete(app_id="STATUS")

	_append_to_log(basic_log_entry)
	return


@post("/log/data/<generator>")
def log_data(generator):
	""" Log endpoint for data generator """
	_log_num(generator, request.params.generated)


def _log_num(name, num):
	""" Log the given number under the given method name """

	number_log_entry = _create_base_log_entry(request.params.vin)

	number_log_entry.complete(
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

	app_id = "COUNTRYCODE"
	position = _get_position_string(crd_x, crd_y)

	# Save request to log
	cc_request_log_entry.complete(
		app_id=app_id,
		log_message="Req",
		gps_position=position
	)

	_append_to_log(cc_request_log_entry)

	country_code = CountryCodeMapper.map(crd_x, crd_y)

	cc_response_log_entry = _create_base_log_entry(request.params.vin)

	# Save response to log
	cc_response_log_entry.complete(
		app_id=app_id,
		log_message="Resp [{}]".format(country_code),
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

	app_id = "POI"
	position = _get_position_string(crd_x, crd_y)

	# Save request to log
	poi_request_log_entry.complete(
		app_id=app_id,
		log_message="Req {}".format(poi_type),
		gps_position=position
	)

	_append_to_log(poi_request_log_entry)

	poi_result = PoiMapper.map(poi_type, crd_x, crd_y)

	poi_response_log_entry = _create_base_log_entry(request.params.vin)
	log_message = "Invalid type {}!".format(poi_type)
	level = LogEntry.LEVEL_ERROR

	if poi_result is not None:
		log_message = "Resp [{}]".format(poi_result)
		level = LogEntry.LEVEL_DEFAULT

	# Save response to log
	poi_response_log_entry.complete(
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

	app_id = "TSPROUTING"
	position = _get_position_string(crd_x, crd_y)

	# Save request to log
	tsp_request_log_entry.complete(
		app_id=app_id,
		log_message="Req [x: {}, y: {}]".format(
			targ_x, targ_y),
		gps_position=position
	)

	_append_to_log(tsp_request_log_entry)

	tsp_message = TspRoutingMapper.map(crd_x, crd_y, targ_x, targ_y)

	tsp_response_log_entry = _create_base_log_entry(request.params.vin)

	if tsp_message == TspRoutingMapper.GOAL_REACHED_MSG:
		tsp_message = "Goal {}/{} reached".format(crd_x, crd_y)

	# Save request to log
	tsp_response_log_entry.complete(
		app_id=app_id,
		log_message="Resp [{}] for [x: {}, y: {}]".format(
			tsp_message, targ_x, targ_y),
		gps_position=position
	)

	_append_to_log(tsp_response_log_entry)


@post("/log/colour")
def log_colour():
	""" Log endpoint with colour input """

	colour_log_entry = _create_base_log_entry(request.params.vin)

	colour_log_entry.complete(
		app_id="COLOUR",
		log_message=request.params.colour
	)

	_append_to_log(colour_log_entry)


@post("/log/get-vins/<num:int>")
def log_num(num):
	""" Log endpoint with number input """

	numbered_log_entry = _create_base_log_entry(request.params.vin)

	numbered_log_entry.complete(
		app_id="GETVINS",
		log_message="Resp {}".format(num))

	_append_to_log(numbered_log_entry)



### DANGER zone


@post("/DANGER/cut-log")
def cut_log():
	""" Cut the log off at the common minimum time of all clients. """

	minimum_time = StateDao.get_current_min_time()

	print("Minimum time is {}. Now processing - this might take some time...".format(
		time.strftime("%Y-%m-%d, %H:%M", time.gmtime(minimum_time))))

	message = StateDao.cut_log()

	return BaseResponse(body=message, status=200)


@post("/DANGER/reset")
def reset():
	""" Rename the log and clear the state. """

	print("Server is resetting...")

	status_msg = ""
	try:
		status_msg = StateDao.reset()
	except ValueError as error:
		status_msg = error.message

	return BaseResponse(body=status_msg, status=200)



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

	client_time = StateDao.get_client_time(identifier)
	time_now = time.time()

	if client_time is None:
		StateDao.set_client_time(identifier, time_now)
		return time_now

	time_choice = random.choice([client_time] * 19 + [client_time + random.randint(3600, 57600)])
	StateDao.set_client_time(identifier, time_choice)

	return time_choice



### Writing to log


def _append_to_log(new_log_entry):
	""" Appends the given string plus a newline to the log file """

	StateDao.append_to_log(new_log_entry)
	return



######################################
### MAIN FLOW: Starting the server ###
######################################


PARSER = argparse.ArgumentParser()
PARSER.add_argument("--quiet", "-q", action="store_true")
ARGS = PARSER.parse_args()

if ARGS.quiet:
	print("Starting server in quiet mode")

StateDao.connect(ARGS.quiet)

run(host="localhost", port=5000, quiet=ARGS.quiet)

StateDao.disconnect(ARGS.quiet)
