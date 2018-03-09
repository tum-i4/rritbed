#!/usr/bin/env python
""" Dir utils """

import os
import time
import uuid
from enum import Enum
import sklearn.svm as sk_svm
from sklearn.externals import joblib


class Dir(object):
	""" Generic directory and path handling. """

	@staticmethod
	def move_file(file_path, target_folder):
		""" Move the given file from /current/path/file_name to target_folder/file_name. """

		file_name = os.path.basename(file_path)
		if not file_name:
			raise ValueError("File path given misses a file name: {}".format(file_path))

		os.rename(file_path, os.path.join(target_folder, file_name))


class LogDir(object):
	""" Log directory and path handling in the IDS. """

	_LOG_DIR = "log"
	_LOG_FILE_PREFIX = "intrusion_"
	_LOG_FILE_SUFFIX = ".log"


	@staticmethod
	def list_log_files():
		""" Return a list of relative paths of all current log files. """
		return _list_files_by_suffix(LogDir.get_log_dir(), LogDir._LOG_FILE_SUFFIX)


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
			LogDir._LOG_FILE_PREFIX + uuid.uuid4().__str__() + LogDir._LOG_FILE_SUFFIX))


	@staticmethod
	def _create_unique_folder_name():
		""" Create a unique name for a backup folder based on the current time. """
		return _create_unique_name(lambda: (
			"logs_until_" + time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())))


	@staticmethod
	def get_log_dir():
		""" Return the log directory. Makes sure the folder exists. """

		log_dir = _get_cwd(_for=LogDir._LOG_DIR)

		if not os.path.lexists(log_dir):
			os.mkdir(log_dir)

		return log_dir


	@staticmethod
	def get_log_path_for(name):
		""" Build a path from the given file name and the log directory. """
		return os.path.join(LogDir.get_log_dir(), name)



class ModelDir(object):
	""" Model directory and path handling in the IDS. """

	_MODEL_DIR = "models"
	_MODEL_FILE_SUFFIX = ".model"


	@staticmethod
	def has_model(app_id):
		""" Check the model directory if there exists a model for the given app_id. """
		return ModelDir.has_models([app_id]) == ModelDir.Found.ALL


	@staticmethod
	def has_models(app_id_list):
		"""
		Check the model directory on disk if there are existing models for each given app_id.
		returns: ModelDir.Found object
		"""

		model_files = ModelDir._list_model_files()
		if not model_files:
			return ModelDir.Found.NONE

		model_names = [os.path.basename(path) for path in model_files]

		# Check if for each app_id a file in the form of app_id.model exists
		found_some = False # OR  init with FALSE
		found_all = True   # AND init with TRUE
		for app_id in app_id_list:
			found_this = ModelDir.get_model_name_for(app_id) in model_names
			found_some |= found_this
			found_all &= found_this

		if found_all:
			return ModelDir.Found.ALL
		elif found_some:
			return ModelDir.Found.SOME

		return ModelDir.Found.NONE


	@staticmethod
	def _list_model_files():
		""" Return a list of relative paths of all current model files. """
		return _list_files_by_suffix(ModelDir.get_model_dir(), ModelDir._MODEL_FILE_SUFFIX)


	@staticmethod
	def save_model(model, app_id, overwrite=False):
		""" Persist the given model for the given app_id on disk. """

		if not isinstance(model, sk_svm.LinearSVC):
			raise ValueError("Models can currently only be of type svm.LinearSVC")

		model_path = ModelDir.get_model_path_for_app_id(app_id)
		if os.path.lexists(model_path):
			if not overwrite:
				raise ValueError("Model file for the given model exists and overwrite is set to False.")
			os.remove(model_path)

		joblib.dump(model, model_path)


	@staticmethod
	def load_model(app_id):
		"""
		Retrieve the model for the given app_id from disk.
		returns: None if no model is present.
		"""

		if not ModelDir.has_model(app_id):
			return None

		model = joblib.load(ModelDir.get_model_path_for_app_id(app_id))

		if not isinstance(model, sk_svm.LinearSVC):
			raise ValueError("Invalid model class retrieved."
				+ " Expected: svm.LinearSVC; Got: {}".format(type(model)))

		return model


	@staticmethod
	def get_model_name_for(app_id):
		""" Return the model name for the given app_id. """
		return "{}{}".format(app_id, ModelDir._MODEL_FILE_SUFFIX)


	@staticmethod
	def get_model_dir():
		""" Return the model directory. """
		return _get_cwd(_for=ModelDir._MODEL_DIR)


	@staticmethod
	def get_model_path_for_app_id(app_id):
		""" Build a path to the given app_id's model file in the model directory. """
		return os.path.join(ModelDir.get_model_dir(), ModelDir.get_model_name_for(app_id))


	# pylint: disable-msg=R0903; (Too few public methods - it's an enum)
	class Found(Enum):
		""" How many results where found. """
		NONE = 0
		SOME = 1
		ALL = 2



### Shared private util methods ###


_IDS_DIR = "ids"


def _list_files_by_suffix(folder, suffix):
	""" Return a list of relative paths of all files in the folder with the given suffix. """

	all_file_paths = [os.path.join(folder, name) for name in os.listdir(folder)]

	if not all_file_paths:
		return []

	result = []
	for file_path in all_file_paths:
		if os.path.isfile(file_path) and file_path.endswith(suffix):
			result.append(file_path)

	return result


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
