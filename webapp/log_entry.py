#!/usr/env/python
""" Log entry """

import json
import time
import uuid

class LogEntry(object):
	""" Container class for log data """

	log_entry = {
		"vin" : "",             # Identifier of the car calling the microservice
		"origin" : "",          # Class name sending this log entry - some.java.method
		"appID" : "",           # Name of the microservice using this
		"level" : "",           # INFO, DEBUG, ...
		"env" : "",             # PROD, INT, TEST
		"log_message" : "",
		"transactionID" : "",   # UUID of the request made to the server
		"logID" : "",           # UUID of this log entry
		"userIDs" : "[null]",   # List of (always one?) ints or [null] in string
		"timeUTC" : "",         # When the logging event has happened
		"timeUnix" : 0,         # !Assumption!: Same time as timeUTC
		"context" : {
			"str" : "",
			"client" : "webapi",
			"method" : ""       # method name - see class
		}

		#"hub" : "EMEA",         # "Region" the server is used for: EMEA
		#"threadID" : "",        # Server worker pool thread id serving this request
		#"import_timestamp" : "",# When was the log imported in kibana
		#"log_lib_version" : "", # Version of the logging library used for storing this event
		#"lib_topic" : "",       # Kibana topic name log was sent to
		#"version" : "1",        # Internal kibana version - constant 1
	}

	def __init__(self, vin, origin, appID,
		level="DEBUG", env="TEST", log_message="", userIDs="[null]", context=None,
		transactionID=None, logID=None, timeUnix=None):
		""" Ctor """

		object.__init__(self)

		self.log_entry.vin = vin
		self.log_entry.origin = origin
		self.log_entry.appID = appID
		self.log_entry.level = level
		self.log_entry.env = env
		self.log_entry.log_message = log_message
		self.log_entry.userIDs = userIDs

		if context is not None:
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
