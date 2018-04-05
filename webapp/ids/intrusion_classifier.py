#!/usr/bin/env python
""" Classifier """

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
		ndarray = self._converter.log_entry_to_ndarray(log_entry, app_id)
		predicted_class = self._models[app_id].predict([ndarray])[0]

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

		found_app_ids, converted_entries = self._stream_convert_entries(
			log_entry_generator, limit, printer)

		self._train_entries(converted_entries, printer)

		printer.prt("Finished training classifiers for {}/{} app ids."
			.format(len(found_app_ids), len(self._converter.app_ids)))


	def _stream_convert_entries(self, log_entry_generator, limit, printer):
		"""
		Read up to <limit> entries from disk and convert them to (app_id, vector, class) tuples.
		returns: found_app_ids (set) and converted_entries (tuple list)
		"""

		found_app_ids = set()
		# Save tuple (app_id, vector, class) for lower memory usage
		converted_entries = []
		for log_entry in log_entry_generator:
			if len(converted_entries) == limit:
				break

			converted_entry = self._log_entry_to_prepared_tuple(log_entry)
			found_app_ids.add(converted_entry[0])
			converted_entries.append(converted_entry)

		return (found_app_ids, converted_entries)


	def _train_entries(self, converted_entries, printer):
		""" Train the app_id based classifiers with the given labelled entries. """

		printer.prt("Starting training with {} entries".format(len(converted_entries)))

		# Ensure each app_id classifier has only samples for normal behaviour to learn from.
		printer.prt("Checking for intruded entries...")
		og_entry_count = len(converted_entries)
		converted_entries[:] = [e for e in converted_entries
			if not self._converter.class_means_intruded(e[2])]

		if len(converted_entries) != og_entry_count:
			printer.prt("Warning! Found intruded data in the input file. {} entries were removed."
				.format(og_entry_count - len(converted_entries)))

		printer.prt("Converting...")
		app_id_datasets = self._prepared_tuples_to_train_dict(converted_entries, printer)

		app_id_number = 1

		for app_id, train_set in app_id_datasets.items():
			printer.prt("({}/{}) Training model for \"{}\": "
				.format(app_id_number, len(app_id_datasets), app_id), newline=False)

			# Load model if it exists already
			if ModelDir.load_model(app_id):
				raise IOError("Found existing model on disk!")

			clf = sk_svm.OneClassSVM(random_state=0)
			printer.prt("Creating and training new model... ", newline=False)

			clf.fit(train_set[0])

			printer.prt("Saving to disk... ", newline=False)
			ModelDir.save_model(clf, app_id, overwrite=True)
			printer.prt("Done! ")

			app_id_number += 1

		self._load_models()


	def score(self, log_entries, do_return=False, squelch_output=False):
		"""
		Score the models' prediction for the given log entries.
		: param do_return : Return a machine-readable { app_id: score } dict.
		"""

		printer = util.prtr.Printer(squelch=squelch_output, name="IC")

		if not self._has_models():
			raise ValueError("The classifier has no trained models! Train first, then score.")

		printer.prt("Starting scoring with {} LogEntry objects.".format(len(log_entries)))

		app_id_datasets = self._log_entries_to_train_dict(log_entries, printer)

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
		prediction_was_correct = []

		for correct_class, prediction in zip(score_classes, predictions):
			is_outlier = self._converter.class_means_intruded(correct_class)
			predicted_as_outlier = self._converter.prediction_means_outlier(prediction)

			prediction_was_correct.append(is_outlier == predicted_as_outlier)

		# Filter for True values
		correct_prediction_count = len(filter(None, prediction_was_correct))
		score = float(correct_prediction_count) / len(prediction_was_correct)
		return score


	def cross_val(self, log_entry_generator, iterations, squelch_output=False):

		limit = 5000000

		printer = util.prtr.Printer(squelch=squelch_output, name="IC")

		printer.prt("Streaming from file up to a maximum of {} entries.".format(limit))
		printer.prt("Loading and converting entries...")

		# converted_entries: [(app_id, vector, class)]
		found_app_ids, converted_entries = self._stream_convert_entries(
			log_entry_generator, limit, printer)

		# TODO
		raise NotImplementedError()

		# TODO filter out anomalous instances?

		app_id_datasets = self._prepared_tuples_to_train_dict(converted_entries, printer)

		scores = {}
		for app_id, (X, y) in app_id_datasets:
			clf = svm.SVC(kernel='linear', C=1)
			scores[app_id] = sk_mod.cross_val_score(clf, X, y, cv=iterations)

		return scores



	### Convert ###


	def _log_entry_to_prepared_tuple(self, log_entry):
		""" Convert the given LogEntry object to a (app_id, vector, class) tuple. """

		app_id = ids_tools.log_entry_to_app_id(log_entry)
		ndarray = self._converter.log_entry_to_ndarray(log_entry, app_id)
		its_class = self._converter.log_entry_to_class(log_entry)

		return (app_id, ndarray, its_class)


	def _prepared_tuples_to_train_dict(self, converted_entries, printer):
		""" Store the given converted entry tuples as { app_id : (X, y) }. """

		printer.prt("Dividing the data per app id...")

		app_id_datasets = {}
		for app_id in self._converter.app_ids:
			app_id_datasets[app_id] = ([], [])

		for converted_entry in converted_entries:
			app_id, ndarray, its_class = converted_entry

			app_id_datasets[app_id][0].append(ndarray)
			app_id_datasets[app_id][1].append(its_class)

		printer.prt("Done.")
		return app_id_datasets


	def _log_entries_to_train_dict(self, log_entries, printer):
		""" Convert the given log entries to { app_id : (X, y) }. """

		printer.prt("Transforming the log data to trainable vectors...")
		converted_entries = [self._log_entry_to_prepared_tuple(e) for e in log_entries]
		return self._prepared_tuples_to_train_dict(converted_entries, printer)



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
