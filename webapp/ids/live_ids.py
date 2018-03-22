#!/usr/bin/env python
""" Live IDS """

import os
import time

from intrusion_classifier import IntrusionClassifier
from ids_classification import Classification
from dir_utils import LogDir, ModelDir


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

		message = "INTRUSION DETECTED. Log file saved at: {}".format(file_path)
		if self._verbose:
			message = "\n!!!\n" + message + "\n!!!\n"

		print(message)


	# pylint: disable-msg=R0201; (Method could be a function)
	def reset_log(self):
		""" Move the intrusion logs to a new sub directory. """

		message = "Intrusion logs: " + LogDir.reset_dir()
		return message


	def reset_models(self):
		""" Move the models to a new sub directory. """

		message = "IDS models: " + ModelDir.reset_dir()
		return message


	@staticmethod
	def _write_intrusion_to_file(log_entry, result):
		"""
		Write the given IdsResult to a new file.
		returns: The relative file path of the log file.
		"""

		time_str = time.strftime("%A %B %d %Y - %H:%M:%S", time.localtime())

		log_folder = LogDir.get_log_dir()

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
