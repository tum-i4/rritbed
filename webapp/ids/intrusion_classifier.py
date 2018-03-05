#!/usr/bin/env python
""" Classifier """

from ids_classification import IdsClassification


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
		returns: An IdsClassification object
		"""

		return IdsClassification.intrusion

		raise NotImplementedError()
