#!/usr/bin/env python
""" Dir utils """

import os
import time
import uuid


class LogDir(object):
	""" Log directory and path handling in the IDS. """

	LOG_DIR = "log"
	LOG_FILE_PREFIX = "intrusion_"
	LOG_FILE_SUFFIX = ".log"


	@staticmethod
	def mk_unique_backup_dir():
		"""
		Mkdir a uniquely named backup folder.
		returns: (folder_name, folder_path)
		"""

		folder_name = LogDir._create_unique_folder_name()
		folder_path = LogDir.get_log_path_for(folder_name)
		os.mkdir(folder_path)

		return(folder_name, folder_path)


	@staticmethod
	def create_unique_log_name():
		""" Create a unique log name based on a UUID. """
		return _create_unique_name(lambda: (
			LogDir.LOG_FILE_PREFIX + uuid.uuid4().__str__() + LogDir.LOG_FILE_SUFFIX))


	@staticmethod
	def _create_unique_folder_name():
		""" Create a unique name for a backup folder based on the current time. """
		return _create_unique_name(lambda: (
			"logs_until_" + time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())))


	@staticmethod
	def get_log_dir():
		""" Return the log directory. """
		return _get_cwd(_for=LogDir.LOG_DIR)


	@staticmethod
	def get_log_path_for(name):
		""" Build a path from the given file name and the log directory. """
		return os.path.join(LogDir.get_log_dir(), name)


class ModelDir(object):
	""" Model directory and path handling in the IDS. """

	MODEL_DIR = "models"


	@staticmethod
	def get_model_dir():
		""" Return the model directory. """
		return _get_cwd(_for=ModelDir.MODEL_DIR)



### Shared private util methods ###


_IDS_DIR = "ids"


def _get_cwd(_for=None):
	"""
	Return the cwd in respect to where the module was loaded.
	: param _for : Optionally the file or folder to be accessed.
	"""

	cwd = "" if os.path.basename(os.getcwd()) == _IDS_DIR else _IDS_DIR
	return os.path.join(cwd, _for) if _for else cwd


def _create_unique_name(name_creator):
	""" Generic name creator method ensuring uniqueness. """

	name = name_creator()
	while os.path.lexists(LogDir.get_log_path_for(name)):
		name = name_creator()

	return name
