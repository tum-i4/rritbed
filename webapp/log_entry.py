#!/usr/env/python
""" Log entry """

import json
import uuid

# pylint: disable-msg=R0903; (Too few public methods (1/2))
class LogEntry(object):
	""" Container class for log data """

	LEVEL_DEFAULT = "DEBUG"
	LEVEL_ERROR = "ERROR"
	ENV_DEFAULT = "TEST"


	vin_field = "vin"
	app_id_field = "app_id"
	level_field = "level"
	gps_position_field = "gps_position"
	log_message_field = "log_message"
	log_id_field = "log_id"
	time_unix_field = "time_unix"

	data = {
		vin_field : "",             # Identifier of the car calling the microservice
		app_id_field : "",          # Name of the microservice using this
		level_field : "",           # INFO, DEBUG, ...
		gps_position_field : "",    # GPS position of car - "12.12312312,42.32321"
		log_message_field : "",
		log_id_field : "",          # UUID of this log entry
		time_unix_field : 0         # !Caution! At COMPANY not the same time as time_utc
	}


	# pylint: disable-msg=R0913; (Too many arguments)
	def __init__(self, vin, app_id, time_unix,
		level=LEVEL_DEFAULT, log_message="", gps_position="",
		log_id=None):
		""" Ctor """

		object.__init__(self)

		self.data[self.vin_field] = vin
		self.data[self.app_id_field] = app_id
		self.data[self.level_field] = level
		self.data[self.log_message_field] = log_message
		self.data[self.gps_position_field] = gps_position

		self.data[self.time_unix_field] = int(time_unix)

		self.data[self.log_id_field] = self._verify_uuid_or_generate_if_none(log_id)


	def set_any(self, vin=None, app_id=None, time_unix=None,
		level=None, log_message=None, gps_position=None,
		log_id=None):
		""" Setter for all fields at once """

		self._set_if_not_none(self.vin_field, vin)
		self._set_if_not_none(self.app_id_field, app_id)
		self._set_if_not_none(self.level_field, level)
		self._set_if_not_none(self.log_message_field, log_message)
		self._set_if_not_none(self.gps_position_field, gps_position)
		self._set_if_not_none(self.time_unix_field, int(time_unix))

		if log_id is not None:
			self._set_if_not_none(self.log_id_field, self._verify_uuid_or_generate_if_none(log_id))


	@staticmethod
	def create_base_entry(vin=None, time_unix=None):
		""" Create an invalid base log entry for step-by-step creation. """
		invalid = "INVALID"
		entry = LogEntry(vin, invalid, invalid, invalid, time_unix)
		return entry


	def complete(self, app_id, vin=None, time_unix=None,
		level=None, log_message=None, gps_position=None,
		log_id=None):
		""" Complete this entry from an invalid base entry to a full log entry. """
		self.set_any(
			vin=vin, app_id=app_id, time_unix=time_unix,
			level=level, log_message=log_message, gps_position=gps_position,
			log_id=log_id)


	def _set_if_not_none(self, field_key, value):
		""" Set the field with the given key to the value specified if that is not None. """
		if value is not None:
			self.data[field_key] = value


	# pylint: disable-msg=R0201; (Method could be a function)
	def _verify_uuid_or_generate_if_none(self, given_id):
		""" Check to see if the id is set, otherwise generates a new UUID. """
		if given_id is None:
			return uuid.uuid4().__str__()

		return uuid.UUID(given_id)


	def get_log_string(self):
		""" Create a log string from this item's log data, sorted by key. """
		return json.dumps(self.data, sort_keys=True)
