#!/usr/bin/env python
""" Classifier """

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
		Train the classifier with the given labelled entries.
		"""

		pass


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
