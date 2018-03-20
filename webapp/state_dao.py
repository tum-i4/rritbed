#!/usr/bin/env python
""" DAO for the STATE """

# pylint: disable-msg=R0902; (Too many instance attributes)

import datetime
import json
import os
import shutil
import time

from log_entry import LogEntry


class StateDao(object):
	""" DAO class for handling the STATE objects """

	_INSTANCE = None


	def __init__(self, verbose, flush_frequency=None, max_entries_in_state=None):
		""" Ctor """

		if StateDao._INSTANCE:
			raise ValueError("DAO is already instantiated!")

		object.__init__(self)

		StateDao._INSTANCE = self
		self._verbose = verbose

		self._state_path = "state"
		self._state_file_name = "state"
		self._state_file_path = os.path.join(self._state_path, self._state_file_name)
		self._log_path = "log"
		self._log_file_name = "log"
		self._log_file_path = os.path.join(self._log_path, self._log_file_name)

		self._curr_min_time = None
		self._client_times = {}
		self._new_log_entries = [] # LogEntry objects

		self._unique_log_file_names = []

		# Auto-flush
		self._flush_frequency = flush_frequency
		self._max_entries_in_state = max_entries_in_state
		self._auto_flush = flush_frequency is not None or max_entries_in_state is not None
		self._last_flush = time.time()


	def __enter__(self):
		""" Initialise this DAO. """

		# Make sure the required directories exist
		for directory_path in [self._state_path, self._log_path]:
			if not os.path.lexists(directory_path):
				os.mkdir(directory_path)

		# List all files in state directory
		files = []
		for (_, _, filenames) in os.walk(self._state_path):
			files.extend(filenames)
			break

		for file_name in files:
			# State not initialised and files exist - load from file
			self._load_state_from_file(file_name)

		if self._verbose:
			print("Loaded state from {} files from disk.".format(len(files)))

		return self


	def __exit__(self, exc_type, exc_value, traceback):
		""" Deinitialising this DAO. """

		self._write_all_to_files()

		if self._verbose:
			print("Successfully saved state to disk.")



	### Interface methods ###


	def cut_log_io(self):
		""" Call cut_log and return an I/O-friendly message with its result. """

		result = self.cut_log()
		if not result:
			return "Log is empty"

		lines_removed, log_length, new_file_path = result

		return "Process finished! Removed {} from the original {} lines.\nSaved file to: {}".format(
			lines_removed, log_length, new_file_path)


	def cut_log(self):
		"""
		Save a copy of the log that is cut at the current minimum time.
		returns: None if no log file exists, else (lines_removed, log_length, new_file_path).
		"""

		if not os.path.lexists(self._log_file_path):
			return None

		new_file_path = self.create_unique_log_file_path()

		self.flush_log()

		shutil.copyfile(self._log_file_path, new_file_path)

		log_lines = []
		with open(new_file_path, "r") as new_log_file:
			log_lines = new_log_file.readlines()

		log_length = len(log_lines)

		current_index = len(log_lines) - 1
		while True:
			entry = json.loads(log_lines[current_index])
			if entry[LogEntry.TIME_UNIX_FIELD] <= self.get_current_min_time():
				break
			log_lines.pop()
			current_index -= 1

		with open(new_file_path, "w") as new_log_file:
			new_log_file.writelines(log_lines)

		return (log_length - len(log_lines), log_length, new_file_path)


	def reset(self):
		"""
		Reset the STATE by deleting the underlying files.\n
		returns: Status message denoting success of underlying operations.
		"""

		self.flush_log()

		status_msg = "Log file: "
		status_msg += self._rename_log_file()
		status_msg += "\nState files: "
		status_msg += self._delete_state_files()

		self._clear_internal_state()

		return status_msg


	def flush_log(self):
		""" Force a write of all new log entries to disk. """

		self._last_flush = time.time()

		number_of_entries = len(self._new_log_entries)

		if number_of_entries == 0:
			return

		print("Flushing {} log entries. Last flush was {}."
			.format(len(self._new_log_entries),
				time.strftime("%H:%M:%S", time.localtime(self._last_flush))))

		# Remove new entries from list and save them to disk
		with open(self._log_file_path, "a") as log_file:
			for _ in range(0, number_of_entries):
				new_log_entry = self._new_log_entries.pop(0)
				log_file.write(new_log_entry.get_log_string() + "\n")


	def get_current_min_time(self):
		""" Getter for the STATE. Reads from disk and updates internal state. """
		return self._curr_min_time


	def get_client_time(self, identifier):
		"""
		Get current time for the specified client. Reads from disk and updates internal state.\n
		returns: Unix time or None for not initialised clients.
		"""

		try:
			return self._client_times[identifier]
		except KeyError:
			self.set_client_time(identifier, None)
			return None


	def set_client_time(self, identifier, new_time):
		""" Setter for the STATE. Updates the internal state and saves to disk. """

		self._client_times[identifier] = new_time

		if self._curr_min_time is None:
			self._curr_min_time = new_time
		else:
			self._curr_min_time = min(self._client_times.values())


	def append_to_log(self, log_entry):
		""" Append the given LogEntry object to the log. """
		self._new_log_entries.append(log_entry)

		if not self._auto_flush:
			return

		do_auto_flush = (
			# max_log was set and is reached
			(self._max_entries_in_state is not None
			and len(self._new_log_entries) >= self._max_entries_in_state)
			or
			# flush_frequency was set and is reached
			(self._flush_frequency is not None
			and self._last_flush + self._flush_frequency <= time.time())
		)

		if do_auto_flush:
			self.flush_log()


	def create_unique_log_file_path(self):
		"""
		Create a unique time-based log file name for log file backups.\n
		Uniqueness is guaranteed for an active session even if no file is created.
		"""
		return self._create_unique_log_file_path()


	def count_log_lines(self):
		""" Count the log lines in the file as well as those kept in-memory. """

		log_line_count = len(self._new_log_entries)

		if not os.path.lexists(self._log_file_path):
			return log_line_count

		with open(self._log_file_path, "r") as log_file:
			lines = log_file.readlines()
			log_line_count += len(lines)

		return log_line_count



	### File access ###


	def _load_state_from_file(self, file_name):
		""" Load the state from file. Differentiates between client and state files. """

		state_from_file = None
		with open(self._get_state_path(file_name), "r") as state_file:
			state_from_file = json.loads(state_file.read())

		if file_name == self._state_file_name:
			self._curr_min_time = state_from_file
		else:
			self._client_times[file_name] = state_from_file


	def _write_all_to_files(self):
		""" Save the internal state to the corresponding files. """

		if not os.path.lexists(self._state_path):
			os.mkdir(self._state_path)

		# Write current minimum time
		with open(self._state_file_path, "w") as state_file:
			state_file.write(json.dumps(self._curr_min_time))

		# Write clients' current time
		for key, value in self._client_times.items():
			with open(self._get_state_path(key), "w") as client_file:
				client_file.write(json.dumps(value))

		# Append new log entries
		self.flush_log()


	def _rename_log_file(self):
		"""
		Rename the log file to a unique time-based name.\n
		returns: Status message denoting success.
		"""

		if not os.path.lexists(self._log_file_path):
			return "File doesn't exist"

		new_file_name = self._create_unique_log_file_path()
		os.rename(self._log_file_path, new_file_name)
		return "File was renamed successfully"


	def _delete_state_files(self):
		"""
		Delete all underlying files.\n
		returns: Status message denoting success.
		"""

		if not os.path.lexists(self._state_path):
			return

		# Delete state file
		StateDao._delete_file_if_existing(
			self._state_file_path)

		# Delete client files
		for client_file_path in self._get_client_file_paths():
			StateDao._delete_file_if_existing(
				client_file_path)

		return "Cleared successfully"


	@staticmethod
	def _delete_file_if_existing(file_path):
		if os.path.lexists(file_path):
			os.remove(file_path)


	### File paths ###


	def _get_client_file_paths(self):
		""" Create the relative paths of all client files. """
		paths = []
		for key in self._client_times:
			paths.append(self._get_state_path(key))
		return paths


	def _get_state_path(self, file_name):
		""" Build a state path to the given file. """
		return os.path.join(self._state_path, file_name)


	def _create_unique_log_file_path(self):
		""" Create a unique log file name for backups. """

		time_unix = time.time()
		new_file_name = self._create_log_file_name_from_time(time_unix)

		while os.path.lexists(new_file_name) or (new_file_name in self._unique_log_file_names):
			time_unix += datetime.timedelta(seconds=1)
			new_file_name = self._create_log_file_name_from_time(time_unix)

		self._unique_log_file_names.append(new_file_name)

		return new_file_name


	def _create_log_file_name_from_time(self, time_unix):
		""" Create a log file name of the format 'log/log_until_2017-12-20_18:08:25'. """
		time_str = time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime(time_unix))
		return self._log_file_path + "_until_" + time_str



	### Helper methods ###


	def _clear_internal_state(self):
		""" Reset all internal fields. """

		self._curr_min_time = None
		self._client_times = {}
		self._new_log_entries = []



if __name__ == "__main__":
	with StateDao(True) as DAO:
		print("yep")
