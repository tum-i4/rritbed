#!/usr/env/python
""" Log entry """

import json
import time
import uuid

# pylint: disable-msg=R0903; (Too few public methods (1/2))
class LogEntry(object):
	""" Container class for log data """

	LEVEL_DEFAULT = "DEBUG"
	LEVEL_ERROR = "ERROR"
	ENV_DEFAULT = "TEST"


	vin_field = "vin"
	origin_field = "origin"
	log_lib_version_field = "log_lib_version"
	app_id_field = "appID"
	level_field = "level"
	env_field = "env"
	log_message_field = "log_message"
	transaction_id_field = "transactionID"
	log_id_field = "logID"
	user_ids_field = "userIDs"
	time_utc_field = "timeUTC"
	time_unix_field = "timeUnix"
	session_id_field = "session_id"
	gps_position_field = "gps_position"
	execution_time_field = "execution_time"

	log_entry = {
		vin_field : "",             # Identifier of the car calling the microservice
		origin_field : "",          # Class name sending this log entry - some.java.method
		log_lib_version_field : "", # Version of the logging library used for storing this event -
									# each microservice has their own
		app_id_field : "",           # Name of the microservice using this
		level_field : "",           # INFO, DEBUG, ...
		env_field : "",             # PROD, INT, TEST
		log_message_field : "",
		transaction_id_field : "",   # UUID of the request made to the server
		log_id_field : "",           # UUID of this log entry
		user_ids_field : "[null]",   # List of (always one?) ints or [null] in string
		time_utc_field : "",        # When the logging event has happened
		time_unix_field : 0,        # !Caution! At COMPANY not the same time as time_utc

		session_id_field : "",
		gps_position_field : "",    # GPS position of car - "12.12312312,42.32321"
		execution_time_field : 0    # How long the execution took
	}


	# pylint: disable-msg=R0913; (Too many arguments)
	def __init__(self, vin, origin, log_lib_version, app_id, time_unix,
		level=LEVEL_DEFAULT, env=ENV_DEFAULT, log_message="", user_ids="[null]", gps_position="",
		transaction_id=None, log_id=None):
		""" Ctor """

		object.__init__(self)

		self.log_entry[self.vin_field] = vin
		self.log_entry[self.origin_field] = origin
		self.log_entry[self.log_lib_version_field] = log_lib_version
		self.log_entry[self.appID_field] = appID
		self.log_entry[self.level_field] = level
		self.log_entry[self.env_field] = env
		self.log_entry[self.log_message_field] = log_message
		self.log_entry[self.userIDs_field] = userIDs
		self.log_entry[self.gps_position_field] = gps_position

		self.log_entry[self.transactionID_field] = self._verify_or_generate_id(transactionID)
		self.log_entry[self.logID_field] = self._verify_or_generate_id(logID)

		self._set_time(time_unix)


	# pylint: disable-msg=R0201; (Method could be a function)
	def _verify_or_generate_id(self, given_id):
		""" Checks to see if the id is set, otherwise generates a new UUID """

		if given_id is None:
			return uuid.uuid4().__str__()

		return given_id


	def _set_time(self, time_unix):
		""" Sets timeUnix and timeUTC fields in this item's log data """

		if time_unix is None:
			time_unix = time.time()

		time_utc_now = time.gmtime(time_unix)

		self.log_entry[self.time_unix_field] = int(time_unix)
		self.log_entry[self.time_utc_field] = time.strftime("%a %b %d %H:%M:%S UTC %Y", time_utc_now)


	def get_log_string(self):
		""" Creates a log string from this item's log data, sorted by key """

		return json.dumps(self.log_entry, sort_keys=True)
