#!/usr/env/python
""" Log entry """

import json
import time
import uuid

class LogEntry(object):
	""" Container class for log data """

	log_entry = {
		"origin" : "",          # Class name sending this log entry - some.java.method
		"log_lib_version" : "", # Version of the logging library used for storing this event
		"hub" : "EMEA",         # "Region" the server is used for: EMEA
		"level" : "",           # INFO, DEBUG, ...
		"import_timestamp" : "",# When was the log imported in kibana
		"appID" : "",           # Name of the microservice using this
		"vin" : "",             # Identifier of the car calling the microservice
		"lib_topic" : "",       # Kibana topic name log was sent to
		"env" : "",             # PROD, INT, TEST
		#"threadID" : "",        # Server worker pool thread id serving this request
		"log_message" : "",
		"transactionID" : "",   # UUID of the request made to the server
		"logID" : "",           # UUID of this log entry
		"userIDs" : "[null]",   # List of (always one?) ints or [null] in string
		#"version" : "1",        # Internal kibana version - constant 1
		"timeUTC" : "",         # When the logging event has happened
		"timeUnix" : 0,         # !Assumption!: Same time as timeUTC
		"context" : {
			"str" : "",
			"client" : "webapi",
			"method" : ""        # method name - see class
		}
	}

	def __init__(self, origin, log_lib_version, import_timestamp, appID,
		vin, lib_topic, log_message, context, transactionID=None, logID=None,
		level="DEBUG", env="TEST", userIDs="[null]", timeUnix=None):
		""" Ctor """

		object.__init__(self)

		self.log_entry.origin = origin
		self.log_entry.log_lib_version = log_lib_version
		self.log_entry.level = level
		self.log_entry.import_timestamp = import_timestamp
		self.log_entry.appID = appID
		self.log_entry.vin = vin
		self.log_entry.lib_topic = lib_topic
		self.log_entry.env = env
		self.log_entry.log_message = log_message
		self.log_entry.userIDs = userIDs
		self.log_entry.context = context

		self.log_entry.transactionID = self.set_or_generate_id(transactionID)
		self.log_entry.logID = self.set_or_generate_id(logID)

		self.set_time(timeUnix)


	def set_or_generate_id(self, given_id):
		""" Checks to see if the id is set, otherwise generates a new UUID """

		if given_id is None:
			return uuid.uuid4()

		return given_id


	def set_time(self, time_unix):
		""" Sets timeUnix and timeUTC fields in this item's log data """

		if time_unix is None:
			time_unix = time.time()

		time_utc_now = time.gmtime(time_unix)

		self.log_entry.timeUnix = str(int(time_unix))
		self.log_entry.timeUTC = time.strftime("%a %b %d %H:%M:%S UTC %Y", time_utc_now)


	def get_log_string(self):
		""" Creates a log string from this item's log data """

		return json.dumps(self.log_entry)
