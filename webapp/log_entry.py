#!/usr/env/python
""" Log entry """

# pylint: disable-msg=R0903,R0913; (Too few public methods, too many arguments)

import json
import time
import uuid


class LogEntry(object):
	""" Container class for log data """

	LEVEL_DEFAULT = "DEBUG"
	LEVEL_ERROR = "ERROR"
	ENV_DEFAULT = "TEST"

	VIN_FIELD = "vin"
	APP_ID_FIELD = "app_id"
	LEVEL_FIELD = "level"
	GPS_POSITION_FIELD = "gps_position"
	LOG_MESSAGE_FIELD = "log_message"
	LOG_ID_FIELD = "log_id"
	TIME_UNIX_FIELD = "time_unix"


	@staticmethod
	def create_base_entry(vin="INVALID", time_unix=None):
		""" Create an invalid base log entry for step-by-step creation. """
		return LogEntry(vin=vin, app_id="INVALID", time_unix=time_unix)



	### Class interface ###


	def __init__(self, vin, app_id,
		level=LEVEL_DEFAULT, log_message="", gps_position="",
		time_unix=None, log_id=None,
		intrusion=None):
		""" Ctor """

		object.__init__(self)

		self.data = {
			LogEntry.VIN_FIELD : "",             # Identifier of the car calling the microservice
			LogEntry.APP_ID_FIELD : "",          # Name of the microservice using this
			LogEntry.LEVEL_FIELD : "",           # INFO, DEBUG, ...
			LogEntry.GPS_POSITION_FIELD : "",    # GPS position of car - "12.12312312,42.32321"
			LogEntry.LOG_MESSAGE_FIELD : "",
			LogEntry.LOG_ID_FIELD : "",          # UUID of this log entry
			LogEntry.TIME_UNIX_FIELD : 0         # !Caution! At COMPANY not the same time as time_utc
		}

		self.intrusion = ""

		time_unix = self._get_current_time_if_none(time_unix)
		log_id = self._generate_uuid_str_if_none(log_id)

		self.set_any(vin=vin, app_id=app_id,
			level=level, log_message=log_message, gps_position=gps_position,
			time_unix=time_unix, log_id=log_id,
			intrusion=intrusion)


	def complete(self, app_id, vin=None, time_unix=None,
		level=None, log_message=None, gps_position=None,
		log_id=None,
		intrusion=None):
		""" Complete this entry from an invalid base entry to a full log entry. """
		self.set_any(vin=vin, app_id=app_id,
			level=level, log_message=log_message, gps_position=gps_position,
			time_unix=time_unix, log_id=log_id,
			intrusion=intrusion)


	def set_any(self, vin=None, app_id=None,
		level=None, log_message=None, gps_position=None,
		time_unix=None, log_id=None,
		intrusion=None):
		""" Setter for all fields at once """

		self._set_if_not_none(LogEntry.VIN_FIELD, vin)
		self._set_if_not_none(LogEntry.APP_ID_FIELD, app_id)
		self._set_if_not_none(LogEntry.LEVEL_FIELD, level)
		self._set_if_not_none(LogEntry.LOG_MESSAGE_FIELD, log_message)
		self._set_if_not_none(LogEntry.GPS_POSITION_FIELD, gps_position)
		self._set_if_not_none(LogEntry.TIME_UNIX_FIELD, time_unix, verifier=self._verify_time)
		self._set_if_not_none(LogEntry.LOG_ID_FIELD, log_id, verifier=self._verify_uuid)

		if intrusion is not None:
			self.intrusion = intrusion


	def get_log_string(self):
		""" Create a log string from this item's log data, sorted by key. """

		result = json.dumps(self.data, sort_keys=True)

		if self.intrusion is not None and self.intrusion != "":
			result += ",{}".format(self.intrusion)

		return result



	### Helper ###


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



	### Generators ###


	@staticmethod
	def _get_current_time_if_none(given_time):
		""" Return the given time or the current time if it's None. """
		return given_time or time.time()


	@staticmethod
	def _generate_uuid_str_if_none(given_uuid):
		""" Return the given UUID or generate one if it's None. """
		return given_uuid or uuid.uuid4().__str__()



	### Verifiers ###


	@staticmethod
	def _verify_time(given_time):
		""" Convert the given time to int. """
		return int(given_time)


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
