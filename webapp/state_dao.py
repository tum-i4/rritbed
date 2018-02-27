#!/usr/bin/env python
""" DAO for the STATE """

import datetime
import json
import os
import shutil
import time

from log_entry import LogEntry


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

	_unique_log_file_names = []


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
	def cut_log():
		""" Save a copy of the log that is cut at the current minimum time. """

		log_file_path = StateDao._get_log_file_path()

		if not os.path.lexists(log_file_path):
			return "Log is empty"

		new_file_path = StateDao.create_unique_log_file_path()

		StateDao._flush_log()

		shutil.copyfile(log_file_path, new_file_path)

		log_lines = []
		with open(new_file_path, "r") as new_log_file:
			log_lines = new_log_file.readlines()

		log_length = len(log_lines)

		current_index = len(log_lines) - 1
		while True:
			entry = json.loads(log_lines[current_index])
			if entry[LogEntry.time_unix_field] <= StateDao.get_current_min_time():
				break
			log_lines.pop()
			current_index -= 1

		with open(new_file_path, "w") as outfile:
			outfile.writelines(log_lines)

		message = "Process finished! Removed {} from the original {} lines.\nSaved file to: {}".format(
			log_length - len(log_lines), log_length, new_file_path)
		return message


	@staticmethod
	def reset():
		"""
		Reset the STATE by deleting the underlying files.\n
		Only possible when connected.\n
		returns: Status message denoting success of underlying operations.
		"""

		if not StateDao._connected:
			raise ValueError("DAO not connected.")

		StateDao.disconnect()

		status_msg = "Log file: "
		status_msg += StateDao._rename_log_file()
		status_msg += "\n State files: "
		status_msg += StateDao._delete_state_files()

		StateDao.connect()

		return status_msg


	@staticmethod
	def create_unique_log_file_path():
		"""
		Create a unique time-based log file name for log file backups.\n
		Uniqueness is guaranteed for an active session even if no file is created.
		"""
		return StateDao._create_unique_log_file_path()




	### Interface methods that are replaced with implementations when connecting DAO. ###


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
		""" Load the state from file. Differentiates between client and state files. """

		state_from_file = None
		with open(StateDao._get_state_path(file_name), "r") as state_file:
			state_from_file = json.loads(state_file.read())

		if file_name == StateDao._state_file_name:
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

		# Append new entries to log
		with open(StateDao._get_log_file_path(), "a") as log_file:
			for new_log_entry in StateDao._new_log_entries:
				log_file.write(new_log_entry.get_log_string() + "\n")

		# Clear log entry list as file is only appended
		StateDao._new_log_entries = []


	@staticmethod
	def _rename_log_file():
		"""
		Rename the log file to a unique time-based name.\n
		returns: Status message denoting success.
		"""

		log_file_path = StateDao._get_log_file_path()

		if not os.path.lexists(log_file_path):
			return "File doesn't exist"
		else:
			new_file_name = StateDao._create_unique_log_file_path()
			os.rename(log_file_path, new_file_name)
			return "File was renamed successfully"


	@staticmethod
	def _delete_state_files():
		"""
		Delete all underlying files.\n
		returns: Status message denoting success.
		"""

		if not os.path.lexists(StateDao._state_path):
			return

		# Delete state file
		StateDao._delete_file_if_existing(
			StateDao._get_state_file_path())

		# Delete client files
		for client_file_path in StateDao._get_client_file_paths():
			StateDao._delete_file_if_existing(
				client_file_path)

		return "Clear was successful"


	@staticmethod
	def _delete_file_if_existing(file_path):
		if os.path.lexists(file_path):
			os.remove(file_path)


	@staticmethod
	def _flush_log():
		""" Force a write of all new log entries to disk. """

		number_of_entries = len(StateDao._new_log_entries)

		# Remove new entries from list and save them to disk
		with open(StateDao._get_log_file_path(), "a") as log_file:
			for _ in range(0, number_of_entries):
				new_log_entry = StateDao._new_log_entries.pop(0)
				log_file.write(new_log_entry.get_log_string() + "\n")


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
		""" Create a unique log file name for backups. """

		time_unix = time.time()
		new_file_name = StateDao._create_log_file_name_from_time(time_unix)

		while os.path.lexists(new_file_name) or (new_file_name in StateDao._unique_log_file_names):
			time_unix += datetime.timedelta(seconds=1)
			new_file_name = StateDao._create_log_file_name_from_time(time_unix)

		StateDao._unique_log_file_names.append(new_file_name)

		return new_file_name


	@staticmethod
	def _create_log_file_name_from_time(time_unix):
		""" Create a log file name of the format 'log/log_until_2017-12-20_18:08:25'. """
		path = StateDao._get_log_file_path()
		return path + "_until_" + time.strftime("%Y-%m-%d_%H:%M:%S", time.gmtime(time_unix))



if __name__ == "__main__":
	StateDao.connect()
	StateDao.disconnect()
