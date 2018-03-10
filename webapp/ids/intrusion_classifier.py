#!/usr/bin/env python
""" Classifier """

import re
import numpy
import sklearn.svm as sk_svm

from log_entry import LogEntry
from functionality.poi_mapper import PoiMapper as PoMa

from dir_utils import ModelDir
from ids_classification import IdsResult, Classification
import ids_tools


class IntrusionClassifier(object):
	""" Classify intrusions rule- and learning-based """

	_INSTANCE = None

	# APP IDS
	_GENERATORS = ["GAUSSIAN", "GUMBEL", "LAPLACE", "LOGISTIC", "PARETO", "RAYLEIGH",
		"UNIFORM", "VONMISES", "WALD", "WEIBULL", "ZIPF"]
	_COLOURS = ["COLOUR"]
	_POSE_CC = "COUNTRYCODE"
	_POSE_POI = "POI"
	_POSE_TSP = "TSPROUTING"
	_POSES = [_POSE_CC, _POSE_POI, _POSE_TSP]

	# POIS
	_INTRUDED_POI_TYPES = ["private home", "nsa hq"]
	_INTRUDED_POI_RESULTS = ["Invalid"]

	# LABELS
	_LEGAL_LABELS = ["normal"]
	_INTRUSION_LABELS_GENS = ["zeroes", "huge-error"]
	_INTRUSION_LABELS_COLRS = ["red"]
	_INTRUSION_LABELS_POIS = ["jump", "illegaltype", "routetoself"]
	_INTRUSION_LABELS = _INTRUSION_LABELS_GENS + _INTRUSION_LABELS_COLRS + _INTRUSION_LABELS_POIS


	@staticmethod
	def get_singleton():
		""" Get the singleton instance. """

		if IntrusionClassifier._INSTANCE:
			return IntrusionClassifier._INSTANCE

		return IntrusionClassifier()


	def __init__(self):
		""" Ctor """

		object.__init__(self)

		if IntrusionClassifier._INSTANCE:
			raise ValueError("Class is already instantiated! Retrieve the instance with get_singleton().")

		self._app_ids = (
			IntrusionClassifier._GENERATORS + IntrusionClassifier._COLOURS + IntrusionClassifier._POSES)
		ids_tools.verify_md5(self._app_ids, "cacafa61f61b645c279954952ac6ba8f")

		self._level_int_mapping = ids_tools.enumerate_to_dict(
			[LogEntry.LEVEL_DEFAULT, LogEntry.LEVEL_ERROR],
			verify_hash="49942f0268aa668e146e533b676f03d0")

		self._poi_type_mapping = ids_tools.enumerate_to_dict(
			[PoMa.restaurants_field, PoMa.gas_stations_field]
			+ IntrusionClassifier._INTRUDED_POI_TYPES,
			verify_hash="f2fba0ed17e382e274f53bbcb142565b")

		self._poi_result_mapping = ids_tools.enumerate_to_dict(
			[PoMa.ita, PoMa.ger, PoMa.frc, PoMa.tot, PoMa.shl, PoMa.arl]
			+ IntrusionClassifier._INTRUDED_POI_RESULTS,
			verify_hash="dd1c18c7188a48a686619fef8007fc64")

		self._label_int_mapping = ids_tools.enumerate_to_dict(
			IntrusionClassifier._LEGAL_LABELS + IntrusionClassifier._INTRUSION_LABELS,
			verify_hash="69a262192b246d16e8411b6db06e237b")

		self._int_label_mapping = ids_tools.flip_dict(
			self._label_int_mapping,
			"c29a85dae460b57fac78db12e72ae24a")

		self._load_models()

		IntrusionClassifier._INSTANCE = self



	### Classify ###


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


	def train(self, log_entries, extend_models=False):
		"""
		Train the app_id based classifiers with the given labelled entries.
		"""

		if not extend_models and self._has_models() != ModelDir.Found.NONE:
			raise ValueError("Extending models was disallowed but there are existing model files on disk.")

		print("Training with {} LogEntry objects".format(len(log_entries)))

		app_id_datasets = {}
		for app_id in self._app_ids:
			app_id_datasets[app_id] = ([], [])

		print("Found {} app ids".format(len(app_id_datasets)))

		# Ensure that all app_ids exist in the dataset
		if (len(app_id_datasets) != len(self._app_ids)
			or any([True for x in self._app_ids if x not in app_id_datasets])):
			raise ValueError("Couldn't find data for every current app_id!")

		for log_entry in log_entries:
			app_id = self._log_entry_to_app_id(log_entry)
			ndarray = self._log_entry_to_ndarray(log_entry, app_id)
			its_class = self._log_entry_to_class(log_entry)

			app_id_datasets[app_id][0].append(ndarray)
			app_id_datasets[app_id][1].append(its_class)

		# Ensure each app_id classifier has samples of all classes to learn from.
		print("Verifying given data...")
		for app_id, train_set in app_id_datasets.items():
			expected_classes = self._get_expected_classes(app_id)
			received_classes = set(train_set[1])
			value_error = ValueError(
				"The given samples for classifier {} don't contain all expected classes.".format(app_id)
				+ " Expected: {}. Received: {}.".format(expected_classes, list(received_classes)))

			if len(expected_classes) != len(received_classes):
				raise value_error
			for exp_class in expected_classes:
				if exp_class not in received_classes:
					raise value_error

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

		self._load_models()
		print("\nTraining completed.")


	def _get_expected_classes(self, app_id):
		""" Return a list of expected classes for the given app_id classifier. """

		labels = IntrusionClassifier._LEGAL_LABELS
		if app_id in IntrusionClassifier._GENERATORS:
			labels += IntrusionClassifier._INTRUSION_LABELS_GENS
		elif app_id in IntrusionClassifier._COLOURS:
			labels += IntrusionClassifier._INTRUSION_LABELS_COLRS
		elif app_id in IntrusionClassifier._POSES:
			labels += IntrusionClassifier._INTRUSION_LABELS_POIS
		else:
			raise ValueError("Invalid app_id given: {}".format(app_id))

		return [self._label_int_mapping[x] for x in labels]



	### Load, check and save model ###


	def _load_models(self):
		"""
		Try to load existing models from the model directory on disk.
		raises: If not all models could be found.
		"""

		if ModelDir.has_models(self._app_ids) != ModelDir.Found.ALL:
			self._models = None
			return

		models = {}
		for app_id in self._app_ids:
			model = ModelDir.load_model(app_id)
			if not model:
				raise IOError("Model for \"{}\" could not be retrieved".format(app_id))
			models[app_id] = model

		assert(len(models) == len(self._app_ids))

		self._models = models


	def _has_models(self):
		"""
		Checks the ModelDir for all current app_ids.
		returns: A ModelDir.Found enum
		"""
		return ModelDir.has_models(self._app_ids)



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
		# Convert gps_position to float
		gps_float = self._gps_position_to_float(data_dict[LogEntry.GPS_POSITION_FIELD], app_id)
		# Convert log_message to float based on app_id
		log_msg_float = self._log_message_to_float(data_dict[LogEntry.LOG_MESSAGE_FIELD], app_id)

		data = [log_msg_float, vin_float, level_int, time_unix]
		if gps_float is not None:
			data += [gps_float]

		result = numpy.array(
			data,
			dtype=numpy.float_,
			order="C")

		self._verify_ndarray(result, app_id)

		return result


	def _vin_to_float(self, vin):
		""" Convert the given VIN to float(aggregate([ord(char), int(rest)])). """

		if len(vin) != 7:
			raise ValueError("Invalid VIN")

		return IntrusionClassifier._aggregate_ints_to_float([ord(vin[0]), int(vin[1:])])


	def _gps_position_to_float(self, gps_position, app_id):
		""" Convert the given GPS position string to (lat, lon). """

		if app_id not in IntrusionClassifier._POSES:
			return None

		# Format: lat,lon
		split = gps_position.split(",")
		if len(split) != 2:
			raise ValueError("Invalid string")

		return IntrusionClassifier._aggregate_ints_to_float([int(split[0]), int(split[1])])


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

		if not isinstance(ndarray, numpy.ndarray) or ndarray.dtype != numpy.float_:
			raise ValueError("Given array is of invalid type.")

		expected_len = 4
		if app_id in IntrusionClassifier._POSES:
			expected_len += 1
		if len(ndarray) != expected_len:
			raise ValueError("Given ndarray is too short. Expected {} elements. Received: {}"
				.format(expected_len, ndarray))

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
		assert(constraints[app_id](ndarray[0]))



	### Util ###


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
