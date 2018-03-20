#!/usr/bin/env python
""" Classifier """

import time

import sklearn.svm as sk_svm

from log_entry import LogEntry

import ids_data
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

		self._app_ids = ids_data.get_app_ids()
		ids_tools.verify_md5(self._app_ids, "cacafa61f61b645c279954952ac6ba8f")

		self._converter = ids_converter.IdsConverter()

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
		ndarray = self._log_entry_to_ndarray(log_entry, app_id)
		predicted_class = self._models[app_id].predict([ndarray])[0]

		classification = Classification.normal
		if self._int_label_mapping[predicted_class] in ids_data.get_intrusion_labels():
			classification = Classification.intrusion

		return IdsResult(classification=classification, confidence=70)



	### Train ###


	def train(self, log_entries, multi_class, extend_models=False, squelch_output=False):
		"""
		Train the app_id based classifiers with the given labelled entries.
		"""

		printer = ids_tools.Printer(squelch=squelch_output, name="IC")

		if not extend_models and self._has_models() != ModelDir.Found.NONE:
			raise ValueError("Extending models was disallowed but there are existing model files on disk.")

		if extend_models and ((self._type == ModelDir.Type.MULTICLASS) != multi_class):
			raise ValueError("Extending models was activated but classifier type does not match train type.")

		printer.prt("Starting training with {} LogEntry objects ({})".format(
			len(log_entries),
			"multi-class" if multi_class else "two-class"))
		start_time = time.time()

		printer.prt("Found all {} app ids".format(len(self._app_ids)))

		app_id_datasets = self._log_entries_to_app_id_train_data_dict(log_entries, multi_class, printer)

		# Ensure that all app_ids exist in the dataset
		if (len(app_id_datasets) != len(self._app_ids)
			or any([True for x in self._app_ids if x not in app_id_datasets])):
			raise ValueError("Couldn't find data for every current app_id!")

		# Ensure each app_id classifier has samples of all classes to learn from.
		printer.prt("Verifying given data...")
		for app_id, train_set in app_id_datasets.items():
			expected_classes = self._get_expected_classes(app_id, multi_class)
			received_classes = set(train_set[1])
			value_error = ValueError(
				"The given samples for classifier {} don't contain all expected classes.".format(app_id)
				+ " Expected: {}. Received: {}.".format(
					[self._int_label_mapping[x] for x in expected_classes],
					[self._int_label_mapping[x] for x in received_classes])
			)

			if len(expected_classes) != len(received_classes):
				raise value_error
			for exp_class in expected_classes:
				if exp_class not in received_classes:
					raise value_error

		model_type = ModelDir.Type.MULTICLASS if multi_class else ModelDir.Type.TWOCLASS
		ModelDir.set_model_type(model_type)

		app_id_count = 1

		for app_id, train_set in app_id_datasets.items():
			printer.prt("({}/{}) Training model for \"{}\""
				.format(app_id_count, len(app_id_datasets), app_id))

			# Load model if it exists already
			clf = ModelDir.load_model(app_id)
			if not clf:
				clf = sk_svm.LinearSVC()
				printer.prt("Creating and training new model...")
			else:
				printer.prt("Model retrieved from disk. Training...")

			clf.fit(train_set[0], train_set[1])

			printer.prt("Saving to disk...")
			ModelDir.save_model(clf, app_id, overwrite=True)
			printer.prt("Done!")

			app_id_count += 1

		self._load_models()

		time_expired = time.time() - start_time
		printer.prt("")
		printer.prt("Training completed in {}.".format(ids_tools.format_time_passed(time_expired)))


	def score(self, log_entries, multi_class, do_return=False, squelch_output=False):
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

		if self._type == ModelDir.Type.NONE:
			raise ValueError("No model type set.")
		elif (self._type == ModelDir.Type.MULTICLASS) != multi_class:
			raise ValueError("Trained models are of invalid type.")

		printer.prt("Starting scoring with {} LogEntry objects ({}).".format(
			len(log_entries),
			"multi-class" if multi_class else "two-class"))

		app_id_datasets = self._log_entries_to_app_id_train_data_dict(log_entries, multi_class, printer)

		app_id_count = 1
		scores = {}

		for app_id, train_set in app_id_datasets.items():
			printer.prt("({}/{}) Scoring model for \"{}\"..."
				.format(app_id_count, len(app_id_datasets), app_id))

			# Load model if it exists already
			clf = ModelDir.load_model(app_id)
			if not clf:
				raise ValueError("Model is missing!")

			score = clf.score(train_set[0], train_set[1])

			printer.prt("Model scored {}.".format(ids_tools.format_percentage(score)))

			scores[app_id] = score
			app_id_count += 1

		total_score = sum(scores.values()) / len(scores)

		printer.prt("")
		printer.prt("Total score: {}".format(ids_tools.format_percentage(total_score)))

		if do_return:
			return scores


	def _log_entries_to_app_id_train_data_dict(self, log_entries, multi_class, printer):
		""" Convert the given log entries to feature vectors and classes per app_id. """

		printer.prt("Transforming the log data to trainable vectors...")

		app_id_datasets = {}
		for app_id in self._app_ids:
			app_id_datasets[app_id] = ([], [])

		for log_entry in log_entries:
			app_id = ids_tools.log_entry_to_app_id(log_entry)
			ndarray = self._log_entry_to_ndarray(log_entry, app_id)
			its_class = self._log_entry_to_class(log_entry, multi_class)

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

		if ModelDir.has_models(self._app_ids) != ModelDir.Found.ALL:
			self._models = None
			return

		models = {}
		for app_id in self._app_ids:
			model = ModelDir.load_model(app_id)
			if not model:
				raise IOError("Model for \"{}\" could not be retrieved".format(app_id))
			models[app_id] = model

		if len(models) != len(self._app_ids):
			raise IOError("Invalid number of model files received.")

		self._models = models
		self._type = ModelDir.load_model_type()


	def _has_models(self):
		"""
		Checks the ModelDir for all current app_ids.
		returns: A ModelDir.Found enum
		"""
		return ModelDir.has_models(self._app_ids)


	@staticmethod
	def reset_models(purge=False):
		""" Reset the models.
		returns: A status message. """
		return ModelDir.reset_dir(purge)
