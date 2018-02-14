#!/usr/bin/env python
""" DAO for the STATE """

import json
import os


class StateDao(object):
	""" Static DAO class for handling the STATE objects """

	_connected = False
	_path = "state"
	_state_file_name = "state"


	_is_init_key = "State initialised"
	_curr_min_key = "Current minimum time"
	_state = {
		_is_init_key: False,
		_curr_min_key: None
	}
	_client_times = {}


	@staticmethod
	def connect(quiet=False):
		""" Load STATE from file. """

		if StateDao._connected:
			raise UserWarning("This method should only be called once!")

		StateDao._connected = True
		StateDao._state[StateDao._is_init_key] = True

		# List all files in state directory
		files = []
		for (_, _, filenames) in os.walk(StateDao._path):
			files.extend(filenames)
			break

		for file_name in files:
			# State not initialised and files exist - load from file
			StateDao._load_state_from_file(file_name)

		if not quiet:
			print("Loaded state from {} files from disk.".format(len(files)))


	@staticmethod
	def disconnect(quiet=False):
		""" Write STATE to file. """

		if not StateDao._connected:
			raise UserWarning("This method should only be called when connected.")

		StateDao._write_state_to_file()

		if not quiet:
			print("Saved state to disk.")


	@staticmethod
	def get_current_min_time():
		""" Getter for the STATE. Reads from disk and updates internal state. """

		StateDao._ensure_state_is_initialised()
		return StateDao._state[StateDao._curr_min_key]


	@staticmethod
	def get_client_time(identifier):
		"""
		Get current time for the specified client. Reads from disk and updates internal state.\n
		returns: Unix time or None for not initialised clients.
		"""

		StateDao._ensure_state_is_initialised()
		try:
			return StateDao._client_times[identifier]
		except KeyError:
			StateDao.set_client_time(identifier, None)
			return None


	@staticmethod
	def set_client_time(identifier, new_time):
		""" Setter for the STATE. Updates the internal state and saves to disk. """

		StateDao._ensure_state_is_initialised()

		StateDao._client_times[identifier] = new_time

		# Initialise or set current minimum time
		if StateDao._state[StateDao._curr_min_key] is None:
			StateDao._state[StateDao._curr_min_key] = new_time
		else:
			StateDao._state[StateDao._curr_min_key] = min(StateDao._client_times.values())


	### Assertion ###


	@staticmethod
	def _ensure_state_is_initialised():
		""" Assert initialised state. """
		assert(StateDao._state[StateDao._is_init_key])


	### File access ###


	@staticmethod
	def _load_state_from_file(file_name):
		is_state_file = file_name == StateDao._state_file_name

		state_from_file = None
		with open(StateDao._get_file_path(file_name), "r") as state_file:
			state_from_file = json.loads(state_file.read())

		if is_state_file:
			StateDao._state[StateDao._curr_min_key] = state_from_file
		else:
			StateDao._client_times[file_name] = state_from_file


	@staticmethod
	def _write_state_to_file():
		""" Save the internal state to the corresponding files. """

		# Write STATE (current minimum time)
		with open(StateDao._get_file_path(StateDao._state_file_name), "w") as state_file:
			state_file.write(json.dumps(StateDao._state[StateDao._curr_min_key]))

		# Write clients (current time)
		for key, value in StateDao._client_times.items():
			with open(StateDao._get_file_path(key), "w") as client_file:
				client_file.write(json.dumps(value))


	@staticmethod
	def _get_file_path(file_name):
		""" Build a file path to the given file. """
		return os.path.join(StateDao._path, file_name)



if __name__ == "__main__":
	StateDao._ensure_state_is_initialised()
