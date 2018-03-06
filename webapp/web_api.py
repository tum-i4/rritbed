#!/usr/bin/env python
""" Web API """

# pylint: disable-msg=E1101; (X has no Y member - of course they do)

# Monkey-patch
from gevent import monkey
monkey.patch_all()

# pylint: disable-msg=C0411,C0413
import argparse
import random
import time
from bottle import post, run, request, BaseResponse

from log_entry import LogEntry
from state_dao import StateDao
from ids.live_ids import LiveIds
from functionality.country_code_mapper import CountryCodeMapper
from functionality.poi_mapper import PoiMapper
from functionality.tsp_routing_mapper import TspRoutingMapper


DAO = None
IDS = None


### API endpoints ###


@post("/log/data/<generator>")
def log_data(generator):
	""" Log endpoint for data generator """
	_log_num(generator)


def _log_num(name):
	""" Log the given number under the given method name. """

	number_log_entry = _create_base_log_entry(request.params.vin)

	number_log_entry.complete(
		app_id=name.upper(),
		log_message=request.params.generated,
		intrusion=request.params.intrusion
	)

	_append_and_detect(number_log_entry)


@post("/get/country-code")
def get_country_code():
	""" Map coordinates to country code and save to log. """

	crd_x = request.params.x
	crd_y = request.params.y

	app_id = "COUNTRYCODE"
	position = _get_position_string(crd_x, crd_y)

	country_code = CountryCodeMapper.map(crd_x, crd_y)

	cc_log_entry = _create_base_log_entry(request.params.vin)

	cc_log_entry.complete(
		app_id=app_id,
		log_message=str(country_code),
		gps_position=position,
		intrusion=request.params.intrusion
	)

	_append_and_detect(cc_log_entry)


@post("/get/poi")
def get_poi():
	""" Map coordinates to POI of given type and save to log. """

	crd_x = request.params.x
	crd_y = request.params.y
	poi_type = request.params.type

	app_id = "POI"
	position = _get_position_string(crd_x, crd_y)

	poi_result = PoiMapper.map(poi_type, crd_x, crd_y)

	poi_log_entry = _create_base_log_entry(request.params.vin)

	log_message = "{}".format(poi_result)
	level = LogEntry.LEVEL_DEFAULT

	if poi_result is None:
		log_message = "Invalid type {}!".format(poi_type)
		level = LogEntry.LEVEL_ERROR

	poi_log_entry.complete(
		app_id=app_id,
		log_message=log_message,
		gps_position=position,
		level=level,
		intrusion=request.params.intrusion
	)

	_append_and_detect(poi_log_entry)


@post("/get/tsp")
def get_tsp_routing():
	""" Map current and goal coordinates to TSP and save to log. """

	crd_x = request.params.x
	crd_y = request.params.y
	targ_x = request.params.targ_x
	targ_y = request.params.targ_y

	app_id = "TSPROUTING"
	position = _get_position_string(crd_x, crd_y)

	tsp_message = TspRoutingMapper.map(crd_x, crd_y, targ_x, targ_y)

	tsp_log_entry = _create_base_log_entry(request.params.vin)

	if tsp_message == TspRoutingMapper.GOAL_REACHED_MSG:
		tsp_message = "Goal {}/{} reached".format(crd_x, crd_y)

	tsp_log_entry.complete(
		app_id=app_id,
		log_message="[{}] for [{},{}]".format(
			tsp_message, # x,y
			targ_x, targ_y),
		gps_position=position,
		intrusion=request.params.intrusion
	)

	_append_and_detect(tsp_log_entry)


@post("/log/colour")
def log_colour():
	""" Log the given colour. """

	colour_log_entry = _create_base_log_entry(request.params.vin)

	colour_log_entry.complete(
		app_id="COLOUR",
		log_message=request.params.colour,
		intrusion=request.params.intrusion
	)

	_append_and_detect(colour_log_entry)



### UTIL zone


@post("/UTIL/flush-log")
def flush_log():
	""" Force a log flush in the DAO. """

	print("Flushing log")

	DAO.flush_log()

	return BaseResponse(body="Log was successfully flushed.", status=200)



### DANGER zone


@post("/DANGER/cut-log")
def cut_log():
	""" Cut the log off at the common minimum time of all clients. """

	minimum_time = DAO.get_current_min_time()

	print("Minimum time is {}. Now processing - this might take some time...".format(
		time.strftime("%Y-%m-%d, %H:%M", time.gmtime(minimum_time))))

	message = DAO.cut_log()

	return BaseResponse(body=message, status=200)


@post("/DANGER/reset")
def reset():
	""" Rename the log and clear the state. """

	print("Server is resetting...")

	status_msg = ""
	try:
		status_msg = DAO.reset()
	except ValueError as error:
		status_msg = error.message

	status_msg += "\n"
	status_msg += IDS.reset_log()

	print(status_msg)

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

	client_time = DAO.get_client_time(identifier)
	time_now = time.time()

	if client_time is None:
		DAO.set_client_time(identifier, time_now)
		return time_now

	time_choice = random.choice([client_time] * 19 + [client_time + random.randint(3600, 57600)])
	DAO.set_client_time(identifier, time_choice)

	return time_choice



### Writing to log


def _append_and_detect(new_log_entry):
	""" Append the given string plus a newline to the log file and detect possible intrusions. """

	DAO.append_to_log(new_log_entry)
	IDS.process(new_log_entry)



######################################
### MAIN FLOW: Starting the server ###
######################################

PARSER = argparse.ArgumentParser()
PARSER.add_argument("--verbose", "-v", action="store_true")
ARGS = PARSER.parse_args()

if not ARGS.verbose:
	print("Starting server in quiet mode")

with StateDao(verbose=ARGS.verbose) as dao:
	DAO = dao
	IDS = LiveIds(verbose=ARGS.verbose)

	run(server="gevent", host="localhost", port=5000, quiet=(not ARGS.verbose))
