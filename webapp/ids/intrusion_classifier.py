#!/usr/bin/env python
""" Classifier """

import md5
import re
import numpy
import sklearn.svm as sk_svm

from log_entry import LogEntry
from functionality.poi_mapper import PoiMapper as PoMa

from dir_utils import ModelDir
from ids_classification import IdsResult, Classification


class IntrusionClassifier(object):
	""" Classify intrusions rule- and learning-based """

	_GENERATORS = ["GAUSSIAN", "GUMBEL", "LAPLACE", "LOGISTIC", "PARETO", "RAYLEIGH",
		"UNIFORM", "VONMISES", "WALD", "WEIBULL", "ZIPF"]
	_COLOURS = ["COLOUR"]
	_POSE_CC = "COUNTRYCODE"
	_POSE_POI = "POI"
	_POSE_TSP = "TSPROUTING"
	_POSES = [_POSE_CC, _POSE_POI, _POSE_TSP]


	def __init__(self):
		""" Ctor """

		object.__init__(self)

		self._app_ids = self._GENERATORS + self._COLOURS + self._POSES
		IntrusionClassifier._verify_md5(self._app_ids, "cacafa61f61b645c279954952ac6ba8f")

		self._level_int_mapping = IntrusionClassifier._enumerate_to_dict(
			[LogEntry.LEVEL_DEFAULT, LogEntry.LEVEL_ERROR],
			verify_hash="49942f0268aa668e146e533b676f03d0")

		self._label_int_mapping = IntrusionClassifier._enumerate_to_dict(
			["normal", "zeroes", "huge-error", "red", "jump", "illegaltype", "routetoself"],
			verify_hash="69a262192b246d16e8411b6db06e237b")

		self._poi_type_mapping = IntrusionClassifier._enumerate_to_dict(
			[PoMa.restaurants_field, PoMa.gas_stations_field],
			verify_hash="0a6d0159ee9e89b34167d7c77c977571")

		self._poi_result_mapping = IntrusionClassifier._enumerate_to_dict(
			[PoMa.ita, PoMa.ger, PoMa.frc, PoMa.tot, PoMa.shl, PoMa.arl],
			verify_hash="a2b714454328ea9fbfb50064b378c147")


	### Classify ###


	# Stateless rules:
	# - Field != == > < value
	# - Field - transformed - equality
	# - Two fields in relation
	# Learning / smart:
	# - Data points are fed

	def classify(self, log_entry):
		"""
		Classify the given log entry and return the classification.
		returns: An IdsResult object
		"""

		# FIRST VERSION: 100 % / 0 %

		# Pass the entry through all systems

		# 1) Rule-based system: If confidence == 100 %: return
		rule_result = self._classify_rule_based(log_entry)
		if rule_result.confidence == 100:
			return rule_result

		# 2) Learning system: If confidence > 60 %: return
		learner_result = self._classify_learner(log_entry)
		if learner_result.confidence > 60:
			return learner_result

		return (rule_result
			if rule_result.confidence > learner_result.confidence
			else learner_result)


	def _classify_rule_based(self, log_entry):
		"""
		Classify the given entry based on pre-defined rules.
		returns: An IdsResult object
		"""

		# Level cannot be ERROR
		if log_entry.data[LogEntry.LEVEL_FIELD] == LogEntry.LEVEL_ERROR:
			return IdsResult(classification=Classification.intrusion, confidence=100)

		return IdsResult(classification=Classification.normal, confidence=0)


	def _classify_learner(self, log_entry):
		"""
		Classify the given entry based on a learning system.
		returns: An IdsResult object
		"""

		# return IdsResult(classification=Classification.normal, confidence=50)

		# TODO
		raise NotImplementedError()



	### Train ###


	def train(self, log_entries):
		"""
		Train the app_id based classifiers with the given labelled entries.
		"""

		print("Training with {} LogEntry objects".format(len(log_entries)))

		app_id_datasets = {}
		for app_id in self._app_ids:
			app_id_datasets[app_id] = ([], [])

		print("Found {} app ids".format(len(app_id_datasets)))

		for log_entry in log_entries:
			app_id = self._log_entry_to_app_id(log_entry)
			ndarray = self._log_entry_to_ndarray(log_entry, app_id)
			its_class = self._log_entry_to_class(log_entry)

			app_id_datasets[app_id][0].append(ndarray)
			app_id_datasets[app_id][1].append(its_class)

		for app_id, train_set in app_id_datasets.items():
			print("Training model for \"{}\"".format(app_id))

			# Load model if it exists already
			clf = ModelDir.load_model(app_id)
			if not clf:
				clf = sk_svm.LinearSVC()
				print("Creating and training new model...")
			else:
				print("Model retrieved from disk. Training...")

			clf.fit(train_set[0], train_set[1])

			print("Saving to disk...")

			ModelDir.save_model(clf, app_id, overwrite=True)

			print("Done!")

		print("\nTraining completed.")



	### Load, check and save model ###


	def _load_models(self, app_id_list):
		"""
		Try to load existing models from the model directory on disk.
		returns: None if loading failed or there are no models.\
		A dict { app_id : model | None } if some or all could be retrieved.
		"""
		raise NotImplementedError()



	### Convert, map, transform ###


	def _log_entry_to_class(self, log_entry):
		""" Map the given LogEntry object to a class to predict. """

		assert(log_entry.intrusion)
		return self._label_int_mapping[log_entry.intrusion]


	def _log_entry_to_app_id(self, log_entry):
		""" Extract and sanitize the app_id from the given LogEntry object. """

		app_id = log_entry.data[LogEntry.APP_ID_FIELD]
		return IntrusionClassifier._strip_app_id(app_id)


	def _log_entry_to_ndarray(self, log_entry, app_id):
		"""
		Convert the given LogEntry object to a learnable vector.
		returns: C-ordered numpy.ndarray (dense) with dtype=float64
		"""

		# We have: vin, app_id, level, log_message, gps_position, time_unix, log_id
		assert(len(log_entry.data) == 7)

		data_dict = log_entry.data
		# Discard log_id (unnecessary) and app_id (it's used for mapping to a classifier)
		# Convert vin to float
		vin_float = self._vin_to_float(data_dict[LogEntry.VIN_FIELD])
		# Keep time_unix as is
		time_unix = data_dict[LogEntry.TIME_UNIX_FIELD]
		# Map level to int
		level_int = self._level_int_mapping[data_dict[LogEntry.LEVEL_FIELD]]
		# Convert gps_position to two floats
		gps_tuple = self._gps_position_to_float_tuple(
			data_dict[LogEntry.GPS_POSITION_FIELD])
		gps_lat = gps_tuple[0]
		gps_lon = gps_tuple[1]
		# Convert log_message to float based on app_id
		log_msg_float = self._log_message_to_float(data_dict[LogEntry.LOG_MESSAGE_FIELD], app_id)

		result = numpy.array(
			[vin_float, level_int, gps_lat, gps_lon, log_msg_float, time_unix],
			dtype=numpy.float_,
			order="C")

		self._verify_ndarray(result, app_id)

		return result


	def _vin_to_float(self, vin):
		""" Convert the given VIN to float(aggregate([ord(char), int(rest)])). """

		if len(vin) != 7:
			raise ValueError("Invalid VIN")

		return IntrusionClassifier._aggregate_ints_to_float([ord(vin[0]), int(vin[1:])])


	def _gps_position_to_float_tuple(self, gps_position):
		""" Convert the given GPS position string to (lat, lon). """

		# Format: lat,lon
		split = gps_position.split(",")
		if len(split) != 2:
			raise ValueError("Invalid string")

		return (float(split[0]), float(split[1]))


	def _log_message_to_float(self, log_message, app_id):
		""" Convert the given log message to a float list based on the given app_id. """

		if app_id not in self._app_ids:
			raise ValueError("Invalid value for app_id given: {}".format(app_id))

		# Generators send "{f}"
		if app_id in IntrusionClassifier._GENERATORS:
			return float(log_message)

		# Colour sends "{i},{i},{i}"
		if app_id in IntrusionClassifier._COLOURS:
			vals = [int(val) for val in log_message.split(",")]
			assert(len(vals) == 3)
			for val in vals:
				assert(val >= 0 and val <= 255)

			# Transform from [0, 255] to [1, 256] to not have zeroes
			vals = [v + 1 for v in vals]

			# Pad to ensure 12,155,1 is different from 121,55,1
			return IntrusionClassifier._aggregate_ints_to_float(vals, pad_zeroes=3)

		# Poses
		assert(app_id in IntrusionClassifier._POSES)

		# Country code string like "DE" or "CH"
		if app_id == IntrusionClassifier._POSE_CC:
			assert(len(log_message) == 2)

			ord_ints = [ord(x) for x in log_message]
			return IntrusionClassifier._aggregate_ints_to_float(ord_ints)

		# POI pair "type,result"
		if app_id == IntrusionClassifier._POSE_POI:
			pair = log_message.split(",")
			assert(len(pair) == 2)

			# Transform from [0,] to [1,] to not have zeroes
			type_int = 1 + self._poi_type_mapping[pair[0]]
			result_int = 1 + self._poi_result_mapping[pair[1]]

			# Make sure we only have single digits as expected
			for val in [type_int, result_int]:
				assert(val >= 1 and val <= 9)

			return IntrusionClassifier._aggregate_ints_to_float([type_int, result_int])

		# Two positions as "{},{},{},{}" (start,end as x,y)
		if app_id == IntrusionClassifier._POSE_TSP:
			coords = [int(coord) for coord in log_message.split(",")]
			assert(len(coords) == 4)
			for coord in coords:
				assert(coord >= 0 and coord < 500)

			# Transfrom from [0, 499] to [1, 500] to not have zeroes
			coords = [c + 1 for c in coords]

			# Pad to ensure 1,100,1,110 is different from 11,1,1,1
			return IntrusionClassifier._aggregate_ints_to_float(coords, pad_zeroes=3)

		raise NotImplementedError("Pose type {} not implemented".format(app_id))


	def _verify_ndarray(self, ndarray, app_id):
		""" Verifies the given ndarray fits the app_id classifier. """

		assert(isinstance(ndarray, numpy.ndarray))
		assert(ndarray.dtype == numpy.float_)
		assert(len(ndarray) == 6)

		constraints = {}

		# No constraint
		for gen in IntrusionClassifier._GENERATORS:
			constraints[gen] = lambda x: True

		# Min: 1,1,1; max: 256,256,256
		for colr in IntrusionClassifier._COLOURS:
			constraints[colr] = lambda x: x >= 1001001 and x <= 256256256

		# For each char: min: 65 ("A"); max: 90 ("Z")
		constraints[IntrusionClassifier._POSE_CC] = lambda x: x >= 6565 and x <= 9090

		# For both ints: [1,9]
		constraints[IntrusionClassifier._POSE_POI] = lambda x: x >= 11 and x <= 99

		# Min: 1,1,1,1; max: 500,500,500,500
		constraints[IntrusionClassifier._POSE_TSP] = lambda x: x >= 1001001001 and x <= 500500500500

		# Check the constraint with the log_message float
		assert(constraints[app_id](ndarray[4]))



	### Util ###


	@staticmethod
	def _enumerate_to_dict(sequence, verify_hash):
		""" Enumerate the given sequence and save the items in a dict as item : index pairs. """

		mapping = {}
		for index, item in enumerate(sequence):
			mapping[item] = index

		IntrusionClassifier._verify_md5(mapping, verify_hash)

		return mapping


	@staticmethod
	def _strip_app_id(app_id):
		""" Strip the given app_id of its ID. """

		# Match indices in the form of _1
		match = re.search(r"\_\d+", app_id)

		if not match:
			return app_id

		# Return app_id without the matched part
		return app_id[:match.start()]


	@staticmethod
	def _aggregate_ints_to_float(list_of_ints, pad_zeroes=None):
		""" Aggregate the given ints as float(intintint). """

		result = ""
		for i in list_of_ints:
			str_i = str(i)

			if pad_zeroes:
				assert(len(str_i) <= pad_zeroes)
				str_i = str_i.zfill(pad_zeroes)

			result += str_i

		return float(result)


	@staticmethod
	def _verify_md5(obj, md5_hex_digest):
		obj_hash = md5.new(str(obj)).hexdigest()
		if obj_hash != md5_hex_digest:
			raise ValueError("Invalid object given. Received: {}".format(obj_hash))
