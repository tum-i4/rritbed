#!/usr/bin/env python
""" DAO for the STATE """

import json
import os


PATH = "state"
STATE_FILE_NAME = "state"

IS_INIT_KEY = "State initialised"
CURR_MIN_KEY = "Current minimum time"

STATE = {
	IS_INIT_KEY: False,
	CURR_MIN_KEY: None
}
CLIENT_TIMES = {}


class StateDao(object):
	""" Static DAO class for handling the STATE objects """

	_connected = False


	@staticmethod
	def connect():
		""" Load STATE from file. """

		if StateDao._connected:
			raise UserWarning("This method should only be called once!")

		STATE[IS_INIT_KEY] = True

		# List all files in state directory
		files = []
		for (_, _, filenames) in os.walk(PATH):
			files.extend(filenames)
			break

		for file_name in files:
			# State not initialised and files exist - load from file
			StateDao._load_state_from_file(file_name)


	@staticmethod
	def disconnect():
		""" Write STATE to file. """

		if not StateDao._connected:
			raise UserWarning("This method should only be called when connected.")

		StateDao._write_state_to_file()


	@staticmethod
	def get_current_min_time():
		""" Getter for the STATE. Reads from disk and updates internal state. """

		StateDao._ensure_state_is_initialised()
		return STATE[CURR_MIN_KEY]


	@staticmethod
	def get_client_time(identifier):
		"""
		Get current time for the specified client. Reads from disk and updates internal state.\n
		returns: Unix time or None for not initialised clients.
		"""

		StateDao._ensure_state_is_initialised()
		try:
			return CLIENT_TIMES[identifier]
		except KeyError:
			StateDao.set_client_time(identifier, None)
			return None


	@staticmethod
	def set_client_time(identifier, new_time):
		""" Setter for the STATE. Updates the internal state and saves to disk. """

		StateDao._ensure_state_is_initialised()

		CLIENT_TIMES[identifier] = new_time

		# Initialise or set current minimum time
		if STATE[CURR_MIN_KEY] is None:
			STATE[CURR_MIN_KEY] = new_time
		else:
			STATE[CURR_MIN_KEY] = min(CLIENT_TIMES.values())


	@staticmethod
	def _load_state_from_file(file_name):
		is_state_file = file_name == STATE_FILE_NAME

		state_from_file = None
		with open(file_name, "r") as state_file:
			state_from_file = json.loads(state_file.read())

		if is_state_file:
			STATE[CURR_MIN_KEY] = state_from_file
		else:
			CLIENT_TIMES[file_name] = state_from_file


	@staticmethod
	def _write_state_to_file():
		""" Save the internal state to the corresponding files. """

		# Write STATE (current minimum time)
		with open(os.path.join(PATH, STATE_FILE_NAME), "w") as state_file:
			state_file.write(json.dumps(STATE[CURR_MIN_KEY]))

		# Write clients (current time)
		for key, value in CLIENT_TIMES.items():
			with open(os.path.join(PATH, key), "w") as client_file:
				client_file.write(json.dumps(value))


	@staticmethod
	def _ensure_state_is_initialised():
		""" Assert initialised state. """
		assert(STATE[IS_INIT_KEY])



if __name__ == "__main__":
	StateDao._ensure_state_is_initialised()
