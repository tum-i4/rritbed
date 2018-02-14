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
	def _ensure_state_is_initialised():
		""" Initialise the STATE by reading from all files or by creating the object. """

		if STATE[IS_INIT_KEY]:
			return

		STATE[IS_INIT_KEY] = True

		# TODO List all files in state directory
		# TODO For each file: If STATE_FILE: set new state
		# TODO                If CLIEN_FILE: set client

		files = []
		for (_, _, filenames) in os.walk(PATH):
			files.extend(filenames)
			break

		for file_name in files:
			# State not initialised and files exist - load from file
			StateDao._init_state_from_file(file_name)


	@staticmethod
	def _init_state_from_file(file_name):
		is_state_file = file_name == STATE_FILE_NAME

		state_from_file = None
		with open(file_name, "r") as state_file:
			state_from_file = json.loads(state_file.read())

		if is_state_file:
			STATE[CURR_MIN_KEY] = state_from_file
		else:
			CLIENT_TIMES[file_name] = state_from_file


if __name__ == "__main__":
	pass # StateDao._init_state()
