#!/usr/bin/env python
""" Live IDS """

import os
import time

from intrusion_classifier import IntrusionClassifier
from ids_classification import Classification
from dir_utils import LogDir


class LiveIds(object):
	""" Live intrusion detection """

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


	def reset_log(self):
		""" Move the found intrusion logs to a new sub directory. """

		message = "Intrusion logs: "
		log_dir = LogDir.get_log_dir()

		if not os.path.lexists(log_dir):
			return message + "Log folder doesn't exist"

		all_files = os.listdir(log_dir)

		if not all_files:
			return message + "Log folder is empty"

		# Create folder
		folder_name, folder_path = LogDir.mk_unique_backup_dir()

		# Move files
		file_count = 0
		for file_name in all_files:
			file_path = LogDir.get_log_path_for(file_name)
			if os.path.isfile(file_path) and file_path.endswith(".log"):
				file_count += 1
				os.rename(file_path, os.path.join(folder_path, file_name))

		return message + "Moved {} file{} to {}".format(
			file_count,
			"s" if file_count > 1 else "",
			folder_name)


	def _write_intrusion_to_file(self, log_entry, result):
		"""
		Write the given IdsResult to a new file.
		returns: The relative file path of the log file.
		"""

		time_str = time.strftime("%A %B %d %Y - %H:%M:%S", time.localtime())

		log_folder = LogDir.get_log_dir()
		if not os.path.lexists(log_folder):
			os.mkdir(log_folder)

		log_file_path = os.path.join(log_folder, LogDir.create_unique_log_name())
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
	def _write_line(file_handle, text):
		""" Write to the file and append a newline. """
		file_handle.write(text + "\n")
