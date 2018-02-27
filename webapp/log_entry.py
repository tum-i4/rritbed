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
	app_id_field = "app_id"
	level_field = "level"
	log_message_field = "log_message"
	log_id_field = "log_id"
	time_unix_field = "time_unix"
	session_id_field = "session_id"
	gps_position_field = "gps_position"

	# execution_time_field = "execution_time"
	# log_lib_version_field = "log_lib_version"
	# env_field = "env"
	# transaction_id_field = "transaction_id"
	# user_ids_field = "user_ids"
	# time_utc_field = "time_utc"

	data = {
		vin_field : "",             # Identifier of the car calling the microservice
		origin_field : "",          # Class name sending this log entry - some.java.method
		app_id_field : "",          # Name of the microservice using this
		level_field : "",           # INFO, DEBUG, ...
		log_message_field : "",
		log_id_field : "",          # UUID of this log entry
		time_unix_field : 0,        # !Caution! At COMPANY not the same time as time_utc

		session_id_field : "",
		gps_position_field : ""     # GPS position of car - "12.12312312,42.32321"

		# execution_time_field : 0,    # How long the execution took
		# log_lib_version_field : "" # Version of the logging library used for storing this event -
									# each microservice has their own
		# env_field : "",             # PROD, INT, TEST
		# transaction_id_field : "",   # UUID of the request made to the server
		# user_ids_field : "[null]",   # List of (always one?) ints or [null] in string
		# time_utc_field : ""        # When the logging event has happened
	}


	# pylint: disable-msg=R0913; (Too many arguments)
	def __init__(self, vin, origin, app_id, time_unix,
		level=LEVEL_DEFAULT, log_message="", gps_position="",
		log_id=None):
		""" Ctor """

		object.__init__(self)

		self.data[self.vin_field] = vin
		self.data[self.origin_field] = origin
		self.data[self.app_id_field] = app_id
		self.data[self.level_field] = level
		self.data[self.log_message_field] = log_message
		self.data[self.gps_position_field] = gps_position

		self.data[self.log_id_field] = self._verify_or_generate_id(log_id)

		self.data[self.time_unix_field] = int(time_unix)


	def set_any(self, vin=None, origin=None, app_id=None, time_unix=None,
		level=None, log_message=None, gps_position=None,
		log_id=None):
		""" Setter for all fields at once """

		self._set_if_not_none(self.vin_field, vin)
		self._set_if_not_none(self.origin_field, origin)
		self._set_if_not_none(self.app_id_field, app_id)
		self._set_if_not_none(self.level_field, level)
		self._set_if_not_none(self.log_message_field, log_message)
		self._set_if_not_none(self.gps_position_field, gps_position)
		self._set_if_not_none(self.time_unix_field, int(time_unix))

		if log_id is not None:
			self._set_if_not_none(self.log_id_field, self._verify_or_generate_id(log_id))


	@staticmethod
	def create_base_entry(vin=None, time_unix=None):
		""" Creates an invalid base log entry for step-by-step creation """
		invalid = "INVALID"
		entry = LogEntry(vin, invalid, invalid, invalid, time_unix)
		return entry


	def complete(self, origin, app_id, vin=None, time_unix=None,
		level=None, log_message=None, gps_position=None,
		log_id=None):
		""" Completes this entry from an invalid base entry to a full log entry """
		self.set_any(
			vin=vin, origin=origin, app_id=app_id, time_unix=time_unix,
			level=level, log_message=log_message, gps_position=gps_position,
			log_id=log_id)


	def _set_if_not_none(self, field_key, value):
		""" Sets the field with the given key to the value specified if that is not None """
		if value is not None:
			self.data[field_key] = value


	# pylint: disable-msg=R0201; (Method could be a function)
	def _verify_or_generate_id(self, given_id):
		""" Checks to see if the id is set, otherwise generates a new UUID """

		if given_id is None:
			return uuid.uuid4().__str__()

		return given_id


	def get_log_string(self):
		""" Creates a log string from this item's log data, sorted by key """

		return json.dumps(self.data, sort_keys=True)
