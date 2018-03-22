#!/usr/bin/env python
""" Classifier """

import sklearn.svm as sk_svm

from log_entry import LogEntry

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
		ndarray = self._converter.log_entry_to_ndarray(log_entry, app_id)
		predicted_class = self._models[app_id].predict([ndarray])[0]

		classification = Classification.normal
		if self._converter.class_means_intruded(predicted_class):
			classification = Classification.intrusion

		return IdsResult(classification=classification, confidence=70)



	### Train ###


	def train(self, log_entry_generator, extend_models=False, squelch_output=False):
		"""
		Train the app_id based classifiers with the given labelled entries.
		Splits up in batches of 1 mio. entries from given generator.
		"""

		if not extend_models and self._has_models() != ModelDir.Found.NONE:
			raise ValueError("Extending models was disallowed but there are existing model files on disk.")

		batch_limit = 1000000

		printer = ids_tools.Printer(squelch=squelch_output, name="IC")

		printer.prt("Streaming from file in batches of {} entries.".format(batch_limit))
		fill_msg = "Filling batch..."
		printer.prt(fill_msg)

		total_entry_count = 0
		app_ids_found = set()
		current_batch = []
		batch_number = 1
		for log_entry in log_entry_generator:
			if len(current_batch) == batch_limit:
				batch_found_app_ids = self._train_batch(current_batch, batch_number, printer)
				app_ids_found.add(batch_found_app_ids)
				current_batch = []
				batch_number += 1
				printer.prt(fill_msg)

			total_entry_count += 1
			current_batch.append(log_entry)

		if current_batch:
			batch_found_app_ids = self._train_batch(current_batch, batch_number, printer)
			app_ids_found.add(batch_found_app_ids)

		printer.prt("\nFinished training with {} batches and a total of {} elements."
			.format(batch_number, total_entry_count))
		printer.prt("Trained models for {}/{} app ids."
			.format(len(app_ids_found), len(self._converter.app_ids)))


	def _train_batch(self, log_entries, batch_number, printer):
		"""
		Train the app_id based classifiers with the given labelled entries.
		"""

		printer.prt("Batch {}: Starting training with {} LogEntry objects"
			.format(batch_number, len(log_entries)))

		# Ensure each app_id classifier has only samples for normal behaviour to learn from.
		printer.prt("Checking for intruded entries...")
		og_entry_count = len(log_entries)
		log_entries[:] = [e for e in log_entries
			if not self._converter.class_means_intruded(self._converter.log_entry_to_class(e))]

		if len(log_entries) != og_entry_count:
			printer.prt("Warning! Found intruded data in the input file. {} entries were removed."
				.format(og_entry_count - len(log_entries)))

		printer.prt("Converting...")
		app_id_datasets = self._log_entries_to_app_id_train_data_dict(log_entries, printer)

		app_id_number = 1

		for app_id, train_set in app_id_datasets.items():
			printer.prt("({}/{}) Training model for \"{}\": "
				.format(app_id_number, len(app_id_datasets), app_id), newline=False)

			# Load model if it exists already
			clf = ModelDir.load_model(app_id)
			if not clf:
				clf = sk_svm.OneClassSVM(random_state=0)
				printer.prt("Creating and training new model... ", newline=False)
			else:
				printer.prt("Model retrieved from disk. Training... ", newline=False)

			clf.fit(train_set[0])

			printer.prt("Saving to disk... ", newline=False)
			ModelDir.save_model(clf, app_id, overwrite=True)
			printer.prt("Done! ")

			app_id_number += 1

		self._load_models()

		return app_id_number


	def score(self, log_entries, do_return=False, squelch_output=False):
		"""
		Score the models' prediction for the given log entries.
		: param do_return : Return a machine-readable { app_id: score } dict.
		"""

		printer = ids_tools.Printer(squelch=squelch_output, name="IC")

		has_models = self._has_models()
		if has_models == ModelDir.Found.NONE:
			raise ValueError("The classifier has no trained models! Train first, then score.")
		if has_models != ModelDir.Found.ALL:
			raise ValueError("Not all models could be found! Partial scoring is not implemented.")

		printer.prt("Starting scoring with {} LogEntry objects.".format(len(log_entries)))

		app_id_datasets = self._log_entries_to_app_id_train_data_dict(log_entries, printer)

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

			# Load model if it exists already
			model = ModelDir.load_model(app_id)
			if not model:
				raise ValueError("Model is missing!")

			score = self._score_outlier_detection(model, score_set)

			printer.prt("Model scored {}.".format(ids_tools.format_percentage(score)))

			scores[app_id] = score
			app_id_count += 1

		total_score = sum(scores.values()) / len(scores)

		printer.prt("")
		printer.prt("Total score: {}".format(ids_tools.format_percentage(total_score)))

		if do_return:
			return scores


	def _score_outlier_detection(self, model, score_set):
		""" Score the given model with the given train_set (labelled). """

		predictions = model.predict(score_set[0])
		prediction_was_correct = []

		for correct_class, prediction in zip(score_set, predictions):
			is_outlier = self._converter.class_means_intruded(correct_class)
			predicted_as_outlier = self._converter.prediction_means_outlier(prediction)

			prediction_was_correct.append(is_outlier == predicted_as_outlier)

		# Filter for True values
		correct_prediction_count = len(filter(None, prediction_was_correct))
		score = float(correct_prediction_count) / len(prediction_was_correct)
		return score


	def _log_entries_to_app_id_train_data_dict(self, log_entries, printer):
		""" Convert the given log entries to feature vectors and classes per app_id. """

		printer.prt("Transforming the log data to trainable vectors...")

		app_id_datasets = {}
		for app_id in self._converter.app_ids:
			app_id_datasets[app_id] = ([], [])

		for log_entry in log_entries:
			app_id = ids_tools.log_entry_to_app_id(log_entry)
			ndarray = self._converter.log_entry_to_ndarray(log_entry, app_id)
			its_class = self._converter.log_entry_to_class(log_entry)

			app_id_datasets[app_id][0].append(ndarray)
			app_id_datasets[app_id][1].append(its_class)

		printer.prt("Done.")
		return app_id_datasets



	### Load, check and save model ###


	def _load_models(self):
		"""
		Try to load existing models from the model directory on disk.
		raises: If not all models could be found.
		"""

		if ModelDir.has_models(self._converter.app_ids) != ModelDir.Found.ALL:
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
		"""
		Checks the ModelDir for all current app_ids.
		returns: A ModelDir.Found enum
		"""
		return ModelDir.has_models(self._converter.app_ids)


	@staticmethod
	def reset_models(purge=False):
		""" Reset the models.
		returns: A status message. """
		return ModelDir.reset_dir(purge=purge)
