#!/usr/bin/env python
""" DAO for the STATE """

import datetime
import json
import os
import time


class StateDao(object):
	""" Static DAO class for handling the STATE objects """

	_connected = False
	_state_path = "state"
	_state_file_name = "state"
	_log_path = "log"
	_log_file_name = "log"

	_curr_min_time = None
	_client_times = {}
	_new_log_entries = [] # LogEntry objects


	@staticmethod
	def connect(quiet=False):
		""" Load STATE from file. """

		if StateDao._connected:
			raise UserWarning("This method should only be called once!")

		StateDao._connected = True

		# Replace interfaces with implementations
		StateDao._replace_interfaces_with_impl()

		# List all files in state directory
		files = []
		for (_, _, filenames) in os.walk(StateDao._state_path):
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

		StateDao._write_all_to_files()

		if not quiet:
			print("Successfully saved state to disk.")


	@staticmethod
	def reset():
		"""
		Reset the STATE by deleting the underlying files.\n
		Only possible when connected.
		"""

		if not StateDao._connected:
			raise ValueError("DAO not connected.")

		StateDao.disconnect()
		StateDao._delete_files()
		StateDao.connect()




	### Interface methods: Are replaced with implementations when connecting DAO. ###


	@staticmethod
	def get_current_min_time():
		""" Getter for the STATE. Reads from disk and updates internal state. """
		StateDao._dao_not_connected_error()


	# pylint: disable-msg=W0613; (Unused argument)
	@staticmethod
	def get_client_time(identifier):
		"""
		Get current time for the specified client. Reads from disk and updates internal state.\n
		returns: Unix time or None for not initialised clients.
		"""
		StateDao._dao_not_connected_error()


	# pylint: disable-msg=W0613; (Unused argument)
	@staticmethod
	def set_client_time(identifier, new_time):
		""" Setter for the STATE. Updates the internal state and saves to disk. """
		StateDao._dao_not_connected_error()


	# pylint: disable-msg=W0613; (Unused argument)
	@staticmethod
	def append_to_log(log_entry):
		""" Append the given LogEntry object to the log. """
		StateDao._dao_not_connected_error()



	### Implementations ###


	@staticmethod
	def _dao_not_connected_error():
		raise ValueError("DAO not connected")


	@staticmethod
	def _replace_interfaces_with_impl():
		""" Switch implementations of interface methods. """

		StateDao.get_current_min_time = StateDao._get_current_min_time_impl
		StateDao.get_client_time = StateDao._get_client_time_impl
		StateDao.set_client_time = StateDao._set_client_time_impl
		StateDao.append_to_log = StateDao._append_to_log_impl


	@staticmethod
	def _get_current_min_time_impl():
		""" Implementation of get_current_min_time() """
		return StateDao._curr_min_time


	@staticmethod
	def _get_client_time_impl(identifier):
		""" Implementation of get_client_time() """

		try:
			return StateDao._client_times[identifier]
		except KeyError:
			StateDao.set_client_time(identifier, None)
			return None


	@staticmethod
	def _set_client_time_impl(identifier, new_time):
		""" Implementation of set_client_time() """

		StateDao._client_times[identifier] = new_time

		if StateDao._curr_min_time is None:
			StateDao._curr_min_time = new_time
		else:
			StateDao._curr_min_time = min(StateDao._client_times.values())


	@staticmethod
	def _append_to_log_impl(log_entry):
		""" Implementation of append_to_log() """
		StateDao._new_log_entries.append(log_entry)



	### File access ###


	@staticmethod
	def _load_state_from_file(file_name):
		is_state_file = file_name == StateDao._state_file_name

		state_from_file = None
		with open(StateDao._get_state_path(file_name), "r") as state_file:
			state_from_file = json.loads(state_file.read())

		if is_state_file:
			StateDao._curr_min_time = state_from_file
		else:
			StateDao._client_times[file_name] = state_from_file


	@staticmethod
	def _write_all_to_files():
		""" Save the internal state to the corresponding files. """

		if not os.path.lexists(StateDao._state_path):
			os.mkdir(StateDao._state_path)

		# Write current minimum time
		with open(StateDao._get_state_file_path(), "w") as state_file:
			state_file.write(json.dumps(StateDao._curr_min_time))

		# Write clients' current time
		for key, value in StateDao._client_times.items():
			with open(StateDao._get_state_path(key), "w") as client_file:
				client_file.write(json.dumps(value))

		# TODO write to log file (append!)
		raise NotImplementedError()
		# LOG_FILE_HANDLE.write(new_log_entry.get_log_string() + "\n")


	@staticmethod
	def _delete_files():
		""" Delete all underlying files. """

		if not os.path.lexists(StateDao._state_path):
			return

		# Delete state file
		StateDao._delete_file_if_existing(
			StateDao._get_state_file_path())

		# Delete client files
		for client_file_path in StateDao._get_client_file_paths():
			StateDao._delete_file_if_existing(
				client_file_path)

		# TODO rename log file
		raise NotImplementedError()


	@staticmethod
	def _delete_file_if_existing(file_path):
		if os.path.lexists(file_path):
			os.remove(file_path)



	### File paths ###


	@staticmethod
	def _get_state_file_path():
		""" Create the relative path of the state file. """
		return StateDao._get_state_path(StateDao._state_file_name)


	@staticmethod
	def _get_client_file_paths():
		""" Create the relative paths of all client files. """
		paths = []
		for key in StateDao._client_times:
			paths.append(StateDao._get_state_path(key))
		return paths


	@staticmethod
	def _get_state_path(file_name):
		""" Build a state path to the given file. """
		return os.path.join(StateDao._state_path, file_name)


	@staticmethod
	def _get_log_file_path():
		""" Create the realative path of the log file. """
		return StateDao._get_log_path(StateDao._log_file_name)


	@staticmethod
	def _get_log_path(file_name):
		""" Build a log path to the given file. """
		return os.path.join(StateDao._log_path, file_name)


	@staticmethod
	def _create_unique_log_file_path():
		""" Creates a unique log file name for backups """

		time_unix = time.time()
		new_file_name = StateDao._create_log_file_name_from_time(time_unix)

		while os.path.isfile(new_file_name):
			time_unix += datetime.timedelta(seconds=1)
			new_file_name = StateDao._create_log_file_name_from_time(time_unix)

		return new_file_name


	@staticmethod
	def _create_log_file_name_from_time(time_unix):
		""" Creates a log file name of the format 'log/log_until_2017-12-20_18:08:25' """
		path = StateDao._get_log_file_path()
		return path + "_until_" + time.strftime("%Y-%m-%d_%H:%M:%S", time.gmtime(time_unix))



if __name__ == "__main__":
	StateDao.connect()
	StateDao.disconnect()
