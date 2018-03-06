#!/usr/bin/env python
""" Classifier """

import numpy
from ids_classification import IdsResult, Classification
from log_entry import LogEntry


class IntrusionClassifier(object):
	""" Classify intrusions rule- and learning-based """

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


	def train(self, log_entries):
		"""
		Train the app based classifiers with the given labelled entries.
		"""

		pass


	@staticmethod
	def _log_entry_to_vector(log_entry):
		"""
		Convert the given LogEntry object to a learnable vector.
		returns: C-ordered numpy.ndarray (dense) with dtype=float64
		"""

		data_dict = log_entry.data
		# Discard vin, log_id (unnecessary)
		# Discard app_id (it's used for mapping to a classifier)
		app_id = data_dict[LogEntry.APP_ID_FIELD]
		# Keep time_unix as is
		time_unix = data_dict[LogEntry.TIME_UNIX_FIELD]
		# Map level to int
		level_int = IntrusionClassifier._level_to_int(data_dict[LogEntry.LEVEL_FIELD])
		# Map gps_position to two floats
		gps_tuple = IntrusionClassifier._gps_position_to_float_tuple(
			data_dict[LogEntry.GPS_POSITION_FIELD])
		gps_lat = gps_tuple[0]
		gps_lon = gps_tuple[1]
		# Map log_message to list of floats based on app_id
		log_msg_floats = IntrusionClassifier._log_message_to_float_list(
			data_dict[LogEntry.LOG_MESSAGE_FIELD], app_id)

		result = numpy.asarray([time_unix, level_int, gps_lat, gps_lon] + log_msg_floats,
			numpy.float_,
			"C")

		IntrusionClassifier._verify_ndarray(result, app_id)

		return result


	@staticmethod
	def _app_id_to_int(app_id):
		app_id_int_mapping = {
			# GENERATORS
			"GAUSSIAN"    :  0,
			"GUMBEL"      :  1,
			"LAPLACE"     :  2,
			"LOGISTIC"    :  3,
			"PARETO"      :  4,
			"RAYLEIGH"    :  5,
			"UNIFORM"     :  6,
			"VONMISES"    :  7,
			"WALD"        :  8,
			"WEIBULL"     :  9,
			"ZIPF"        : 10,
			# COLOUR
			"COLOUR"      : 11,
			# POSE
			"COUNTRYCODE" : 12,
			"POI"         : 13,
			"TSPROUTING"  : 14
		}

		app_id = IntrusionClassifier._strip_app_id(app_id)
		return app_id_int_mapping[app_id]


	@staticmethod
	def _strip_app_id(app_id):
		raise NotImplementedError()


	@staticmethod
	def _level_to_int(label):
		raise NotImplementedError()


	@staticmethod
	def _gps_position_to_float_tuple(gps_position):
		raise NotImplementedError()


	@staticmethod
	def _log_message_to_float_list(log_message, app_id):
		raise NotImplementedError()


	@staticmethod
	def _label_to_int(label):
		""" Map the given label to the corresponding integer for its class. """

		label_int_mapping = {
			"normal"      : 0,
			# COLOUR
			"red"         : 1,
			# GENERATOR
			"zeroes"      : 2,
			"huge-error"  : 3,
			# POSE
			"jump"        : 4,
			"illegaltype" : 5,
			"routetoself" : 6
		}

		return label_int_mapping[label]


	@staticmethod
	def _verify_ndarray(ndarray, app_id):
		raise NotImplementedError()
