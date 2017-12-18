#!/usr/bin/env python
""" Web API """

import os.path
import uuid
import time
from bottle import post, run, template, request, BaseResponse

log_file_name = "log"

### API endpoints ###

@post("/log")
def log():
	""" Default log endpoint with no arguments """

	with open(log_file_name, "a") as outfile:
		outfile.write(get_log_string())
	return


### Helper methods ###


def get_log_string():
	""" Creates a log string """

	time_unix_now = time.time()
	time_utc_now = time.gmtime(time_unix_now)

	log_message = ("\"origin\" : \"some.java.class\", // service"
		"\"abcd_version\" : \"5.2.3\","
		"\"hub\" : \"ABCD\","
		"\"level\" : \"DEBUG\", // Info, Alert"
		"\"timestamp\" : \"2017-10-12T01:12:12.123Z\","
		"\"appID\" : \"ABCD\","
		"\"vin\" : \"" + "A192738" + "\", // ##ROS##"
		"\"abcd_topic\" : \"abcd_svds\","
		"\"environment\" : \"PROD\", // DEV (development), PROD (customer)"
		"\"timeUTC\" : \"" + time.strftime("%a %b %d %H:%M:%S UTC %Y", time_utc_now) + "\","
		"\"context\" : {"
		"	\"str\" : {"
		"	\"client\" : \"webapi\","
		"	\"method\" : \"METHOD NAME SEE CLASS\""
		"	}"
		"},"
		"\"threadID\" : \"http-bla-bla-bla\","
		"\"logMessage\" : \"getXForUser returned 0 vins\","
		"\"transactionID\" : \"" + uuid.uuid4().__str__() + "\","
		"\"logID\" : \"" + uuid.uuid4().__str__() + "\","
		"\"version\" : \"1\","
		"\"timeUnix\" : " + str(int(time_unix_now)))
	return ("{" + log_message + "}")


run(host="localhost", port=5000)
