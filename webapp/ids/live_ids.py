#!/usr/bin/env python
""" Live IDS """

import os
import time
import uuid

from intrusion_classifier import IntrusionClassifier
from ids_classification import IdsResult, Classification

class LiveIds(object):
	""" Live intrusion detection """

	LOG_DIR = "log"
	LOG_FILE_PREFIX = "intrusion_"
	LOG_FILE_SUFFIX = ".log"

	def __init__(self, verbose):
		""" Ctor """

		object.__init__(self)

		self._verbose = verbose
		self.classifier = IntrusionClassifier()


	def process(self, log_entry):
		""" Process the given entry. Outputs a warning when the detection was successful. """

		result = self.classifier.classify(log_entry)
		if result.classification == Classification.normal and result.confidence > 0:
			return

		file_path = self._write_intrusion_to_file(log_entry, result)

		if self._verbose:
			print("\n!!!\nINTRUSION DETECTED. See log file at: {}\n!!!\n".format(file_path))


	def _write_intrusion_to_file(self, log_entry, result):
		"""
		Write the given IdsResult to a new file.
		returns: The relative file path of the log file.
		"""

		time_str = time.strftime("%a %b %d %Y - %H:%M:%S", time.localtime())

		log_folder = LiveIds._get_log_dir()
		if not os.path.lexists(log_folder):
			os.mkdir(log_folder)

		log_file_path = os.path.join(log_folder, LiveIds._create_unique_log_name())
		with open(log_file_path, mode="w") as log_file:
			LiveIds._write_line(log_file, "Intrusion detected | {}".format(time_str))
			LiveIds._write_line(log_file, "")
			LiveIds._write_line(log_file, "Classification: {}".format(result.classification.name))
			LiveIds._write_line(log_file, "Confidence: {} %".format(result.confidence))
			LiveIds._write_line(log_file, "")
			LiveIds._write_line(log_file, "Data received:")
			LiveIds._write_line(log_file, log_entry.get_log_string())

		return log_file_path


	@staticmethod
	def _create_unique_log_name():
		""" Create a unique log name based on a UUID. """

		# pylint: disable-msg=C0111; (Missing method docstring)
		def build_log_name():
			return LiveIds.LOG_FILE_PREFIX + uuid.uuid4().__str__() + LiveIds.LOG_FILE_SUFFIX

		name = build_log_name()
		while os.path.lexists(os.path.join(LiveIds._get_log_dir(), name)):
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
