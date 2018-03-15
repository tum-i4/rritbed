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


	@staticmethod
	def read_lines(file_path):
		""" Return all lines in the given file. Removes the line terminating character. """

		result = []
		with open(file_path) as file_handle:
			for line in file_handle:
				# Remove the newline character
				result.append(line[:-1])

		return result


class LogDir(object):
	""" Log directory and path handling in the IDS. """

	_LOG_DIR = "log"
	_LOG_FILE_PREFIX = "intrusion_"
	_LOG_FILE_SUFFIX = ".log"


	@staticmethod
	def reset_dir():
		""" Move all model files to a new, unique sub directory and return a status message. """

		file_list = LogDir._list_log_files()
		folder_name, folder_path = LogDir._mk_unique_backup_dir()
		return _reset_dir(file_list, folder_name, folder_path)


	@staticmethod
	def _list_log_files():
		""" Return a list of relative paths of all current log files. """
		return _list_files_by_suffix(LogDir.get_log_dir(), LogDir._LOG_FILE_SUFFIX)


	@staticmethod
	def _mk_unique_backup_dir():
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
		return _get_cwd(_for=LogDir._LOG_DIR, mk_if_nonexistent=True)


	@staticmethod
	def get_log_path_for(name):
		""" Build a path from the given file name and the log directory. """
		return os.path.join(LogDir.get_log_dir(), name)



class ModelDir(object):
	""" Model directory and path handling in the IDS. """

	_MODEL_DIR = "models"
	_MODEL_FILE_SUFFIX = ".model"
	_TYPE_FILE_SUFFIX = ".modeltype"


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
	def reset_dir():
		""" Move all model related files to a new, unique sub directory and return a status message. """

		file_list = ModelDir._list_model_files(include_type_file=True)
		folder_name, folder_path = ModelDir._mk_unique_backup_dir()
		return _reset_dir(file_list, folder_name, folder_path)


	@staticmethod
	def _list_model_files(include_type_file=False):
		""" Return a list of relative paths of all current model files. """

		model_files = _list_files_by_suffix(ModelDir.get_model_dir(), ModelDir._MODEL_FILE_SUFFIX)
		if include_type_file:
			model_files += ModelDir._list_type_files()

		return model_files


	@staticmethod
	def _list_type_files():
		""" Return a list of relative paths of all current type files. """
		return _list_files_by_suffix(ModelDir.get_model_dir(), ModelDir._TYPE_FILE_SUFFIX)


	@staticmethod
	def _mk_unique_backup_dir():
		"""
		Mkdir a uniquely named backup folder.
		returns: (folder_name, folder_path)
		"""

		folder_name = ModelDir._create_unique_folder_name()
		folder_path = ModelDir.get_model_path_for_file(folder_name)
		os.mkdir(folder_path)

		return(folder_name, folder_path)


	@staticmethod
	def _create_unique_folder_name():
		""" Create a unique name for a model backup folder based on the current time. """
		return _create_unique_name(lambda: (
			"model_backup_at_" + time.strftime("%Y-%m-%d_%H:%M:%S", time.localtime())))


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
	def set_model_type(model_type):
		""" Save the given model type for the current models. """

		if not isinstance(model_type, ModelDir.Type):
			raise ValueError("Argument model_type must be of type ModelDir.Type")

		if model_type == ModelDir.Type.NONE:
			raise ValueError("Type NONE can't be saved.")

		if ModelDir._list_type_files():
			raise IOError("Type file exists! Make sure you only set the type once.")

		type_file_name = ModelDir._get_type_file_str(model_type)

		open(ModelDir.get_model_path_for_file(type_file_name + ModelDir._TYPE_FILE_SUFFIX), "a"
			).close()


	@staticmethod
	def load_model_type():
		""" Load the ModelDir.Type and return it. """

		type_files = ModelDir._list_type_files()

		if len(type_files) > 1:
			raise IOError("Invalid number of type files found.")

		if not type_files:
			return ModelDir.Type.NONE

		type_file_name = os.path.basename(type_files[0]).split(os.path.extsep)[0]
		return ModelDir.Type(type_file_name)


	@staticmethod
	def _get_type_file_str(model_type):
		""" Convert the given model type to the corresponding file name. """

		if not isinstance(model_type, ModelDir.Type):
			raise ValueError("Invalid type given.")
		if model_type == ModelDir.Type.NONE:
			raise ValueError("ModelDir.Type.NONE has no file!")

		return model_type.value + ModelDir._TYPE_FILE_SUFFIX


	@staticmethod
	def get_model_name_for(app_id):
		""" Return the model name for the given app_id. """
		return "{}{}".format(app_id, ModelDir._MODEL_FILE_SUFFIX)


	@staticmethod
	def get_model_dir():
		""" Return the model directory. Makes sure the folder exists. """
		return _get_cwd(_for=ModelDir._MODEL_DIR, mk_if_nonexistent=True)


	@staticmethod
	def get_model_path_for_app_id(app_id):
		""" Build a path to the given app_id's model file in the model directory. """
		return ModelDir.get_model_path_for_file(ModelDir.get_model_name_for(app_id))


	@staticmethod
	def get_model_path_for_file(file_name):
		""" Build a path to the given file in the model directory. """
		return os.path.join(ModelDir.get_model_dir(), file_name)


	# pylint: disable-msg=R0903; (Too few public methods - it's an enum)
	class Found(Enum):
		""" How many results where found. """
		NONE = 0
		SOME = 1
		ALL = 2


	class Type(Enum):
		""" What type the current models have. """
		NONE = ""
		TWOCLASS = "twoclass"
		MULTICLASS = "multiclass"



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


def _reset_dir(file_list, folder_name, folder_path):
	""" Move the files to the given folder and return a status message. """

	if not file_list:
		return "Log folder is empty"

	# Move files
	for file_path in file_list:
		Dir.move_file(file_path, folder_path)

	return "Moved {} file{} to {}".format(
		len(file_list),
		"s" if len(file_list) > 1 else "",
		folder_name)


def _get_cwd(_for=None, mk_if_nonexistent=False):
	"""
	Return the cwd in respect to where the module was loaded.
	: param _for : Optionally the file or folder to be accessed.
	"""

	cwd = "" if os.path.basename(os.getcwd()) == _IDS_DIR else _IDS_DIR
	the_dir = os.path.join(cwd, _for) if _for else cwd

	if mk_if_nonexistent and not os.path.lexists(the_dir):
		os.mkdir(the_dir)

	return the_dir


def _create_unique_name(name_creator):
	""" Generic name creator method ensuring uniqueness. """

	name = name_creator()
	while os.path.lexists(LogDir.get_log_path_for(name)):
		name = name_creator()

	return name
