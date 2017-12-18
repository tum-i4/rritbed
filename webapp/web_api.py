#!/usr/bin/env python
from bottle import post, run, template, request, response
import uuid
import time

@post("/log")
def log():
	response.status = 200
	with open("log.txt", "a") as outfile:
		outfile.write(get_log_string())
	return


def get_log_string():
	""" Creates a log string with the given arguments """

	time_unix_now = time.time()
	time_utc_now = time.gmtime(time_unix_now)

	return ("""{
		"\"origin\" : \"some.java.class\", // service
		\"abcd_version\" : \"5.2.3\",
		\"hub\" : \"ABCD\",
		\"level\" : \"DEBUG\", // Info, Alert
		\"timestamp\" : \"2017-10-12T01:12:12.123Z\",
		\"appID\" : \"ABCD\",
		\"vin\" : \"{vin}\", // ##ROS##
		\"abcd_topic\" : \"abcd_svds\",
		\"environment\" : \"PROD\", // DEV (development), PROD (customer
		\"timeUTC\" : \"{time_utc}\",
		\"context\" : {
			\"str\" : {
			\"client\" : \"webapi\",
			\"method\" : \"METHOD NAME SEE CLASS\"
			}
		},
		\"threadID\" : \"http-bla-bla-bla\",
		\"logMessage\" : \"getXForUser returned 0 vins\",
		\"transactionID\" : \"{transaction_uuid}\",
		\"logID\" : \"{log_uuid}\",
		\"version\" : \"1\",
		\"timeUnix\" : {time_unix}
	}""").format(transaction_uuid=uuid.uuid4(),
		log_uuid=uuid.uuid4(),
		time_utc=time_utc_now,
		time_unix=time_unix_now,
		vin="A192738")


run(host="localhost", port=5000)
