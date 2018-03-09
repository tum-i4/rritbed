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
		return _create_unique_name(lambda: (
			LogDir.LOG_FILE_PREFIX + uuid.uuid4().__str__() + LogDir.LOG_FILE_SUFFIX))


	@staticmethod
	def create_unique_folder_name():
		""" Create a unique name for a backup folder based on the current time. """
		return _create_unique_name(lambda: (
			"logs_until_" + time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())))


	@staticmethod
	def get_log_dir():
		""" Return the log directory. """
		return _get_cwd(for_=LogDir.LOG_DIR)


	@staticmethod
	def get_log_path_for(name):
		""" Build a path from the given file name and the log directory. """
		return os.path.join(LogDir.get_log_dir(), name)



### Shared private util methods ###


def _get_cwd(for_=None):
	"""
	Return the cwd in respect to where the module was loaded.
	: param for_ : Optionally the file or folder to be accessed.
	"""

	cwd = "" if os.path.basename(os.getcwd()) == IDS_DIR else IDS_DIR
	return os.path.join(cwd, for_) if for_ else cwd


def _create_unique_name(name_creator):
	""" Generic name creator method ensuring uniqueness. """

	name = name_creator()
	while os.path.lexists(LogDir.get_log_path_for(name)):
		name = name_creator()

	return name
