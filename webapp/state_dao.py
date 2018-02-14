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

	@staticmethod
	def get_client_time(identifier):
		"""
		Getter for the STATE. Reads from disk and updates internal state.\n
		raises: KeyError if identifier does not have a STATE.
		"""

		StateDao._ensure_state_is_initialised()

		return CLIENT_TIMES[identifier]


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

		with open(os.path.join(PATH, STATE_FILE_NAME), "w") as state_file:
			state_file.write(json.dumps(STATE[CURR_MIN_KEY]))

		with open(os.path.join(PATH, identifier), "w") as client_file:
			client_file.write(json.dumps(CLIENT_TIMES[identifier]))


	@staticmethod
	def _ensure_state_is_initialised():
		""" Initialise the STATE by reading from all files or by creating the object. """

		if STATE[IS_INIT_KEY]:
			return

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
	def _load_state_from_file(file_name):
		is_state_file = file_name == STATE_FILE_NAME

		state_from_file = None
		with open(file_name, "r") as state_file:
			state_from_file = json.loads(state_file.read())

		if is_state_file:
			STATE[CURR_MIN_KEY] = state_from_file
		else:
			CLIENT_TIMES[file_name] = state_from_file


if __name__ == "__main__":
	StateDao._ensure_state_is_initialised()
