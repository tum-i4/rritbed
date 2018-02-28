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


	def __init__(self, vin, app_id,
		level=LEVEL_DEFAULT, log_message="", gps_position="",
		time_unix=None, log_id=None):
		""" Ctor """

		object.__init__(self)

		self.data[self.vin_field] = vin
		self.data[self.app_id_field] = app_id
		self.data[self.level_field] = level
		self.data[self.log_message_field] = log_message
		self.data[self.gps_position_field] = gps_position

		self.data[self.time_unix_field] = self._verify_time_or_generate_if_none(time_unix)
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

		# The verification would generate values for None - so it's only triggered if a value was given.
		if time_unix is not None:
			self._set_if_not_none(self.time_unix_field, self._verify_time_or_generate_if_none(time_unix))

		if log_id is not None:
			self._set_if_not_none(self.log_id_field, self._verify_uuid_or_generate_if_none(log_id))


	@staticmethod
	def create_base_entry(vin=None, time_unix=None):
		""" Create an invalid base log entry for step-by-step creation. """
		entry = LogEntry(vin=vin, app_id="INVALID", time_unix=time_unix)
		return entry


	def complete(self, app_id, vin=None, time_unix=None,
		level=None, log_message=None, gps_position=None,
		log_id=None):
		""" Complete this entry from an invalid base entry to a full log entry. """
		self.set_any(
			vin=vin, app_id=app_id, time_unix=time_unix,
			level=level, log_message=log_message, gps_position=gps_position,
			log_id=log_id)


	def _set_if_not_none(self, field_key, value, verifier=None):
		"""
		Set the field with the given key to the value specified if that is not None.\n
		verifier: Optional verification method.
		"""

		if value is None:
			return

		if verifier is not None:
			value = verifier(value)

		self.data[field_key] = value


	@staticmethod
	def _get_current_time_if_none(given_time):
		""" Return the given time or the current time if it's None. """
		return given_time or time.time()


	@staticmethod
	def _verify_time(given_time):
		""" Convert the given time to int. """
		return int(given_time)


	@staticmethod
	def _generate_uuid_str_if_none(given_uuid):
		""" Return the given UUID or generate one if it's None. """
		return given_uuid or uuid.uuid4().__str__()


	@staticmethod
	def _verify_uuid(given_uuid):
		""" Convert the given object to a UUID string if it's not yet one. """

		if isinstance(given_uuid, str):
			# Verify the given string is well-formed
			uuid.UUID(given_uuid)
			return given_uuid

		if isinstance(given_uuid, uuid.UUID):
			return given_uuid.__str__()

		raise ValueError("Given object is neither a string nor a UUID object.")


	def get_log_string(self):
		""" Create a log string from this item's log data, sorted by key. """
		return json.dumps(self.data, sort_keys=True)
