#!/usr/bin/env python
""" Classifier """

import sklearn.metrics as sk_met
import sklearn.model_selection as sk_mod
import sklearn.svm as sk_svm

from log_entry import LogEntry
import util.fmtr
import util.prtr

import ids_tools
import ids_converter
from dir_utils import ModelDir
from ids_classification import IdsResult, Classification


class IntrusionClassifier(object):
	""" Classify intrusions rule- and learning-based """

	_INSTANCE = None


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

		self._converter = ids_converter.IdsConverter()

		self._int_label_mapping = ids_tools.flip_dict(
			self._converter.label_int_mapping,
			verify_hash="c29a85dae460b57fac78db12e72ae24a")

		self._load_models()

		IntrusionClassifier._INSTANCE = self



	### Classify ###


	def classify(self, log_entry):
		"""
		Classify the given log entry and return the classification.
		returns: An IdsResult object
		"""

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


	@staticmethod
	def _classify_rule_based(log_entry):
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

		if self._models is None:
			raise IOError("Some or all model files are missing.")

		app_id = ids_tools.log_entry_to_app_id(log_entry)
		vector = self._converter.log_entry_to_vector(app_id, log_entry)

		predicted_class = self._models[app_id].predict([vector])[0]

		classification = Classification.normal
		if self._converter.class_means_intruded(predicted_class):
			classification = Classification.intrusion

		return IdsResult(classification=classification, confidence=70)



	### Train ###


	def train(self, log_entry_generator, squelch_output=False):
		"""
		Train the app_id based classifiers with the given labelled entries.
		Only loads up to 5 mio entries to ensure the process does not consume too much memory.
		"""

		if self._has_models():
			raise ValueError("There are existing model files on disk.")

		limit = 5000000

		printer = util.prtr.Printer(squelch=squelch_output, name="IC")

		printer.prt("Streaming from file up to a maximum of {} entries.".format(limit))
		printer.prt("Loading and converting entries...")

		ids_entries_dict = self._read_convert_entries(log_entry_generator, limit)

		self._train_entries(ids_entries_dict, printer)

		printer.prt("Finished training classifiers for {}/{} app ids."
			.format(len(ids_entries_dict), len(self._converter.app_ids)))


	def _read_convert_entries(self, log_entry_generator, limit):
		"""
		Read up to <limit> entries from disk and convert them to (app_id, vector, class) tuples.
		returns: found_app_ids (set) and converted_entries (tuple list)
		"""

		# Read up to <limit> entries
		log_entries = []
		for log_entry in log_entry_generator:
			if len(log_entries) == limit:
				break

			log_entries.append(log_entry)

		# Convert entries to { app_id : (X, y) } dict
		ids_entries_dict = self._converter.log_entries_to_ids_entries_dict(log_entries)
		return ids_entries_dict


	def _train_entries(self, ids_entries_dict, printer):
		""" Train all app_id based classifiers with the given labelled entries. """

		app_id_number = 1
		for app_id, ids_entries in ids_entries_dict.items():
			printer.prt("({}/{}) Training model for \"{}\": "
				.format(app_id_number, len(ids_entries_dict), app_id), newline=False)

			# Check for existing model
			if ModelDir.has_model(app_id):
				raise IOError("Found existing model on disk!")

			self._train_entries_per_app(app_id, ids_entries, printer)

			app_id_number += 1


	def _train_entries_per_app(self, app_id, ids_entries, printer):
		""" Train this app_id based classifier with the given labelled entries. """

		printer.prt("Starting training with {} entries".format(len(ids_entries)))

		# Ensure the classifier has only samples for normal behaviour to learn from.
		printer.prt("Checking for intruded entries...")
		og_entry_count = len(ids_entries)
		ids_entries[:] = [e for e in ids_entries
			if not self._converter.class_means_intruded(e.vclass)]

		if len(ids_entries) != og_entry_count:
			printer.prt("Warning! Found intruded data in the input file. {} entries were removed."
				.format(og_entry_count - len(ids_entries)))

		printer.prt("Converting...")
		# pylint: disable-msg=C0103; (Invalid variable name)
		X_train, _ = self._converter.ids_entries_to_X_y(app_id, ids_entries)

		printer.prt("Creating and training new model... ", newline=False)
		clf = sk_svm.OneClassSVM(random_state=0)
		clf.fit(X_train)

		printer.prt("Saving... ", newline=False)
		self._models[app_id] = clf
		ModelDir.save_model(clf, app_id, overwrite=True)
		printer.prt("Done! ")


	def score(self, log_entries, do_return=False, squelch_output=False):
		"""
		Score the models' prediction for the given log entries.
		: param do_return : Return a machine-readable { app_id: score } dict.
		"""

		printer = util.prtr.Printer(squelch=squelch_output, name="IC")

		if not self._has_models():
			raise ValueError("The classifier has no trained models! Train first, then score.")

		printer.prt("Starting scoring with {} LogEntry objects.".format(len(log_entries)))

		app_id_datasets = self._converter.log_entries_to_train_dict(log_entries, printer)

		# Verify
		printer.prt("Verifying data...")
		for app_id, score_set in app_id_datasets.items():
			expected_classes = self._converter.get_expected_classes(app_id)
			received_classes = set(score_set[1])
			if (len(expected_classes) != len(received_classes)
				or any([x not in received_classes for x in expected_classes])):
				printer.prt("Didn't receive all classes for scoring of {}.".format(app_id))

		app_id_count = 1
		scores = {}

		for app_id, score_set in app_id_datasets.items():
			printer.prt("({}/{}) Scoring model for \"{}\"..."
				.format(app_id_count, len(app_id_datasets), app_id))

			# Load model from disk
			model = ModelDir.load_model(app_id)
			if not model:
				raise ValueError("Model is missing!")

			score = self._score_outlier_detection(model, score_set)

			printer.prt("Model scored {}.".format(util.fmtr.format_percentage(score)))

			scores[app_id] = score
			app_id_count += 1

		total_score = sum(scores.values()) / len(scores)

		printer.prt("")
		printer.prt("Total score: {}".format(util.fmtr.format_percentage(total_score)))

		if do_return:
			return scores


	def _score_outlier_detection(self, model, score_set):
		""" Score the given model with the given (X,y) set. """

		score_entries, score_classes = score_set
		predictions = model.predict(score_entries)
		accuracy = sk_met.accuracy_score(y_true=score_classes, y_pred=predictions)
		return accuracy



	### Load, check and save model ###


	def _load_models(self):
		"""
		Try to load existing models from the model directory on disk.
		raises: If not all models could be found.
		"""

		if not self._has_models():
			self._models = None
			return

		models = {}
		for app_id in self._converter.app_ids:
			model = ModelDir.load_model(app_id)
			if not model:
				raise IOError("Model for \"{}\" could not be retrieved".format(app_id))
			models[app_id] = model

		if len(models) != len(self._converter.app_ids):
			raise IOError("Invalid number of model files received.")

		self._models = models


	def _has_models(self):
		""" Checks the ModelDir for all current app_ids. """
		return ModelDir.has_models(self._converter.app_ids)


	@staticmethod
	def reset_models(purge=False):
		""" Reset the models.
		returns: A status message. """
		return ModelDir.reset_dir(purge=purge)
