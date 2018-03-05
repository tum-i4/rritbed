#!/usr/bin/env python
""" Live IDS """

import os
import time
import uuid
from enum import Enum

class LiveIds(object):
	""" Live intrusion detection """

	LOG_DIR = "log"
	LOG_FILE_PREFIX = "intrusion_"
	LOG_FILE_SUFFIX = ".log"

	def __init__(self):
		""" Ctor """

		object.__init__(self)


	def process(self, log_entry):
		""" Process the given entry. Outputs a warning when the detection was successful. """

		classification = self._classify(log_entry)
		if classification == IdsClassification.normal:
			return

		# Assert after most returns are done for improved performance; the above check still works.
		assert(isinstance(classification, IdsClassification))

		self._print_and_log_intrusion(log_entry, classification)

		raise NotImplementedError()


	def _classify(self, log_entry):
		"""
		Classify the given log entry and return the classification.
		returns: An IdsClassification object
		"""

		raise NotImplementedError()


	def _print_and_log_intrusion(self, log_entry, classification):
		""" Print a warning and log the given intrusion in a new file. """

		time_now = time.time()

		log_file_path = LiveIds._get_log_file_path(LiveIds._create_unique_log_name())
		with open(log_file_path, mode="w") as log_file:
			log_file.write("Intrusion detected at ")
		print("INTRUSION DETECTED. See log file: {}")

		raise NotImplementedError()


	@staticmethod
	def _create_unique_log_name():
		""" Create a unique log name based on a UUID. """

		# pylint: disable-msg=C0111; (Missing method docstring)
		def build_log_name():
			return LiveIds.LOG_FILE_PREFIX + uuid.uuid4().__str__

		name = build_log_name()
		while os.path.lexists(LiveIds._get_log_file_path(name)):
			name = build_log_name()

		return name


	@staticmethod
	def _get_log_dir():
		""" Build a path from the given file name and the log folder. """

		target_dir = LiveIds.LOG_DIR

		ids_dir = "ids"
		if os.path.basename(os.getcwd()) != ids_dir:
			target_dir = os.path.join(ids_dir, target_dir)

		return target_dir


	@staticmethod
	def _write_line(file_handle, text):
		""" Write to the file and append a newline. """
		file_handle.write(text + "\n")


# pylint: disable-msg=R0903; (Too few public methods)
class IdsClassification(Enum):
	""" IDS classification results """

	normal = 0
	intrusion = 1
