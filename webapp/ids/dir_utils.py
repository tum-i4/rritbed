#!/usr/bin/env python
""" Dir utils """

import os
import time
import uuid


IDS_DIR = "ids"


class LogDir(object):
	""" Log directory and path handling in the IDS. """

	LOG_DIR = "log"
	LOG_FILE_PREFIX = "intrusion_"
	LOG_FILE_SUFFIX = ".log"


	@staticmethod
	def create_unique_log_name():
		""" Create a unique log name based on a UUID. """
		return LogDir.create_unique_name(lambda: (
			LogDir.LOG_FILE_PREFIX + uuid.uuid4().__str__() + LogDir.LOG_FILE_SUFFIX))


	@staticmethod
	def create_unique_backup_folder_name():
		""" Create a unique name for a backup folder based on the current time. """
		return LogDir.create_unique_name(lambda: (
			"logs_until_" + time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())))


	@staticmethod
	def create_unique_name(name_creator):
		""" Generic name creator method ensuring uniqueness. """

		name = name_creator()
		while os.path.lexists(LogDir.get_log_path_for(name)):
			name = name_creator()

		return name


	@staticmethod
	def get_log_dir():
		""" Return the log directory. """

		target_dir = LogDir.LOG_DIR

		if os.path.basename(os.getcwd()) != IDS_DIR:
			target_dir = os.path.join(IDS_DIR, target_dir)

		return target_dir


	@staticmethod
	def get_log_path_for(name):
		""" Build a path from the given file name and the log directory. """
		return os.path.join(LogDir.get_log_dir(), name)
