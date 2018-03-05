#!/usr/bin/env python
""" Classifier """

from ids_classification import IdsResult, Classification


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

		# Pass the entry through all systems
		# FIRST ITERATION: 100 % / 0 %
		# 1) Rule-based system: If intrusion was detected: return
		# 2) Learning system: If confidence > 60 %: return

		rule_result = self._classify_rule_based(log_entry)
		if rule_result.confidence > 70:
			return rule_result

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
		raise NotImplementedError()


	def _classify_learner(self, log_entry):
		"""
		Classify the given entry based on a learning system.
		returns: An IdsResult object
		"""
		raise NotImplementedError()
