#!/usr/env/python
""" Log entry """

import json
import time

class LogEntry(object):

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
		"threadID" : "",        # Server worker pool thread id serving this request
		"log_message" : "",
		"transactionID" : "",   # UUID of the request made to the server
		"logID" : "",           # UUID of this log entry
		"userIDs" : "[null]",   # List of (always one?) ints or [null] in string
		"version" : "1",        # Internal kibana version - constant 1
		"timeUTC" : "",         # When the logging event has happened
		"timeUnix" : 0,         # !Assumption!: Same time as timeUTC
		"context" : {
			"str" : "",
			"client" : "webapi",
			"method" : ""        # method name - see class
		}
	}

	def __init__(self, origin, log_lib_version, level="DEBUG", import_timestamp, appID, 
		vin, lib_topic, env="TEST", threadID=1, log_message, transactionID,
		logID, userIDs="[null]", version=1, timeUnix=None, context):
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
		self.log_entry.threadID = threadID
		self.log_entry.log_message = log_message
		self.log_entry.transactionID = transactionID
		self.log_entry.logID = logID
		self.log_entry.userIDs = userIDs
		self.log_entry.version = version
		self.log_entry.context = context

		self.set_time(timeUnix)


	def set_time(self, timeUnix):
		""" Sets timeUnix and timeUTC fields in this item's log data """

		time_unix_now = time.time()

		if timeUnix is not None:
			time_unix_now = timeUnix
		
		time_utc_now = time.gmtime(time_unix_now)

		self.log_entry.timeUnix = str(int(time_unix_now))
		self.log_entry.timeUTC = time.strftime("%a %b %d %H:%M:%S UTC %Y", time_utc_now)


	def get_log_string(self):
		""" Creates a log string from this item's log data """
		
		return json.dumps(self.log_entry)
