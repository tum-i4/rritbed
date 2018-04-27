#!/usr/bin/env python
""" Runnable experiments. All respect the ModuleInterface interface. """

# pylint: disable-msg=R0903, C0103; (Too few public methods, invalid name)

import time
import random

import sklearn.ensemble as sk_ens
import sklearn.svm as sk_svm
import sklearn.model_selection as sk_mod

from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter
from ids.ids_one_hot_vs_mapping_converter import OneHotVsMappingConverter
from ids.ids_entry import IdsEntry
import ids.ids_tools as ids_tools
from log_entry import LogEntry
import util.seqr


ITEM_LIMIT = 5000000


class ModuleInterface(object):
	""" Interface for all submodules. """

	@staticmethod
	def run(experiment):
		""" abstract run() method """
		raise NotImplementedError


### Module helpers ###


def verify_ids_entries(ids_entries, app_id, printer):
	""" Check for content and type. """

	if not ids_entries:
		printer.prt("No input data for {}".format(app_id))
		return

	if not isinstance(ids_entries[0], IdsEntry):
		raise TypeError("Given list does not contain IdsEntry objects.")


### Experiments ###


class OneHotVsMapping(ModuleInterface):
	"""
	Experiment:
	Classifier trained with one-hot encoding
	vs. classifier trained with labelling.
	Regards app_ids:
	"""


	@staticmethod
	def run(experiment):

		log_entries = []

		for line in Dir.yield_lines(experiment.file_path, ITEM_LIMIT):
			log_entry = LogEntry.from_log_string(line)
			log_entries.append(log_entry)

		experiment.entries = log_entries

		# Exp 1: map
		time_before_map = time.time()
		OneHotVsMapping.handle_log_entries("MAP", OneHotVsMappingConverter(), log_entries, experiment)
		# Exp 2: one-hot
		time_after_map_before_one_hot = time.time()
		OneHotVsMapping.handle_log_entries("OHOT", IdsConverter(), log_entries, experiment)
		time_after_all = time.time()

		time_for_map = time_after_map_before_one_hot - time_before_map
		time_for_one_hot = time_after_all - time_after_map_before_one_hot

		timing_lines = [
			"Benchmark result | %s entries processed | OneClassSVM classifier" % len(log_entries),
			"",
			"Mapping: %s" % util.fmtr.format_time_passed(time_for_map),
			"One-hot: %s" % util.fmtr.format_time_passed(time_for_one_hot)
		]

		experiment.add_result_file("time_map_vs_onehot", timing_lines)


	@staticmethod
	def handle_log_entries(name, converter, log_entries, experiment):
		""" Handle the given log entries by converting them with the given converter and then scoring. """

		ids_entry_dict = converter.log_entries_to_ids_entries_dict(log_entries, binary=True)

		for app_id, ids_entries in util.seqr.yield_items_in_key_order(ids_entry_dict):
			verify_ids_entries(ids_entries, app_id, experiment.storer_printer)

			first = ids_entries[0]
			experiment.storer_printer.prt("I am %s:%s, and my first entry looks like:" % (name, app_id))
			experiment.storer_printer.prt("%s | %s" % (first.app_id, first.vector))

			training, scoring = ids_tools.ids_entries_to_train_test(ids_entries)
			X_train, _ = converter.ids_entries_to_X_y(training)
			X_test, y_true = converter.ids_entries_to_X_y(scoring)

			classifier = sk_svm.OneClassSVM()
			classifier.fit(X_train)
			y_pred = classifier.predict(X_test)
			experiment.visualise_store(name, app_id, classifier, y_true, y_pred)


class CleanTrainingVsDistorted(ModuleInterface):
	"""
	Experiment:
	Classifier trained on clean data versus
	classifier trained on various distributions of intruded data.
	"""

	@staticmethod
	def run(experiment):
		experiment.storer_printer.prt("Reading entries...")
		# ids_entries: { app_id, vector, my_class }
		ids_entries_dict = experiment.read_convert(experiment.file_path)

		experiment.storer_printer.prt("Running experiment...")

		CleanTrainingVsDistorted.run_cycle(ids_entries_dict, 0, experiment)
		CleanTrainingVsDistorted.run_cycle(ids_entries_dict, 10, experiment)
		CleanTrainingVsDistorted.run_cycle(ids_entries_dict, 20, experiment)
		CleanTrainingVsDistorted.run_cycle(ids_entries_dict, 30, experiment)
		CleanTrainingVsDistorted.run_cycle(ids_entries_dict, 40, experiment)
		CleanTrainingVsDistorted.run_cycle(ids_entries_dict, 50, experiment)

		experiment.add_result_file("run_iterations", ["Ran iterations with 0, 10, 20, 30, 40, 50 %"])

		experiment.storer_printer.prt("Done.")


	@staticmethod
	def run_cycle(ids_entries_dict, percentage_intruded_training, experiment):
		""" Full cycle for all apps with the given percentage. """

		if percentage_intruded_training < 0:
			raise ValueError("percentage_intruded_training needs to be > 0")
		elif percentage_intruded_training == 0:
			experiment.storer_printer.prt("Scoring clean classifier...")
		else:
			experiment.storer_printer.prt("Scoring distorted classifier (%s %%)..."
				% percentage_intruded_training)

		for app_id, ids_entries in util.seqr.yield_items_in_key_order(ids_entries_dict):
			CleanTrainingVsDistorted.run_cycle_for_app(
				ids_entries, app_id, percentage_intruded_training, experiment)


	@staticmethod
	def run_cycle_for_app(ids_entries, app_id, percentage_intruded_training, experiment):
		""" One app with the given percentage. """

		verify_ids_entries(ids_entries, app_id, experiment.storer_printer)

		training, scoring = CleanTrainingVsDistorted.custom_train_test_split(
			ids_entries, percentage_intruded_training)

		X_train, _ = IdsConverter().ids_entries_to_X_y(training)
		X_test, y_true = IdsConverter().ids_entries_to_X_y(scoring)

		classifier = sk_svm.OneClassSVM()
		name = CleanTrainingVsDistorted.get_name(percentage_intruded_training)

		classifier.fit(X_train)
		y_pred = classifier.predict(X_test)
		experiment.visualise_store(name, app_id, classifier, y_true, y_pred)


	@staticmethod
	def custom_train_test_split(ids_entries, target_pct_intruded_training):
		""" Split in train/test and ensure target_pct... of intruded entries in the training set. """

		if any([entry.vclass not in [1, -1] for entry in ids_entries[:100]]):
			raise ValueError("Given entries are not valid IdsEntry objects!")

		entries_normal = []
		entries_intruded = []
		for ids_entry in ids_entries:
			if ids_tools.is_inlier(ids_entry.vclass):
				entries_normal.append(ids_entry)
			else:
				entries_intruded.append(ids_entry)

		if len(entries_intruded) < 500 or len(entries_normal) < 1000:
			raise ValueError("Too few intruded/normal entries")

		percentage_intruded = (len(entries_intruded) / float(len(entries_normal)))
		ids_tools.verify_percentage_intruded(percentage_intruded)

		# Always select the smaller set as a baseline. Choose the test size as the smaller set's size.
		# 50 % will be normal, 50 % intruded.
		half_size_test = int(float(min(len(entries_normal), len(entries_intruded)))/2)

		percentage_for_normal = float(half_size_test) / len(entries_normal)
		percentage_for_intruded = float(half_size_test) / len(entries_intruded)

		assert(percentage_for_normal <= 0.5 and percentage_for_intruded <= 0.5)

		remaining_normal, test_normal = sk_mod.train_test_split(
			entries_normal, test_size=percentage_for_normal)
		remaining_intruded, test_intruded = sk_mod.train_test_split(
			entries_intruded, test_size=percentage_for_intruded)

		scoring_entries = test_normal + test_intruded
		random.shuffle(scoring_entries)

		# Prevent future errors
		entries_normal = None
		entries_intruded = None

		# Remaining are 50 % of the smaller set and > 50 % of the bigger set.
		# Check current proportion

		remaining_count = len(remaining_normal) + len(remaining_intruded)

		relative_size_intruded = float(len(remaining_intruded)) / remaining_count

		excessive_percent_normal = 0.0
		excessive_percent_intruded = 0.0

		# If we have too few intruded entries, sample from normal entries
		if relative_size_intruded < target_pct_intruded_training:
			# Target size derived from number of intruded entries
			target_size = (1 / target_pct_intruded_training) * len(remaining_intruded)
			normal_entries_needed = target_size - len(remaining_intruded)
			normal_entries_excessive = len(remaining_normal) - normal_entries_needed
			excessive_percent_normal = float(normal_entries_excessive) / len(remaining_normal)
		# If we have too many, sample from them
		elif relative_size_intruded > target_pct_intruded_training:
			# Target size derived from number of normal entries
			target_size = (1 / (1 - target_pct_intruded_training)) * len(remaining_normal)
			intruded_entries_needed = target_size - len(remaining_normal)
			intruded_entries_excessive = len(remaining_intruded) - intruded_entries_needed
			excessive_percent_intruded = float(intruded_entries_excessive) / len(remaining_intruded)

		training_normal, _ = sk_mod.train_test_split(
			remaining_normal, test_size=excessive_percent_normal)
		training_intruded, _ = sk_mod.train_test_split(
			remaining_intruded, test_size=excessive_percent_intruded)

		achieved_percentage_intruded = (
			float(len(training_intruded))
			/ (len(training_normal) + len(training_intruded)))

		# No more than 1 % error
		assert(abs(target_pct_intruded_training - achieved_percentage_intruded) < 0.01)

		training_entries = training_normal + training_intruded
		random.shuffle(training_entries)

		# Ensure we calculated everything correctly.
		assert(len(training_entries) >= max(len(remaining_normal), len(remaining_intruded)))
		assert(min(len(training_entries), len(scoring_entries)) > 1000)

		return (training_entries, scoring_entries)


	@staticmethod
	def get_name(percentage_intruded_training):
		""" Create a name based on the percentage of intruded entries. """

		if percentage_intruded_training == 0:
			return "CLeN"
		else:
			return "DS{:2}".format(percentage_intruded_training)


class AllVsSpecSvmVsIso(ModuleInterface):
	"""
	Experiment:
	Classifier trained on all vs specialised classifiers,
	OneClassSVM vs IsolationForest.
	"""


	@staticmethod
	def run(experiment):
		experiment.storer_printer.prt("Scoring classifier trained with all samples...")
		AllVsSpecSvmVsIso.handle_all(experiment)

		experiment.storer_printer.prt("Done. Scoring individual classifier...")
		# ids_entries: { app_id, vector, my_class }
		ids_entries_dict = experiment.read_convert(experiment.file_path)

		for app_id, ids_entries in util.seqr.yield_items_in_key_order(ids_entries_dict):
			AllVsSpecSvmVsIso.handle_app(app_id, ids_entries, experiment)

		experiment.storer_printer.prt("Done.")


	@staticmethod
	def handle_all(experiment):
		""" Full flow for a one-fits-all classifier. """

		from ids.TEMP_IDS_CONVERTER import IdsConverter as TEMPCONVERTER
		converter = TEMPCONVERTER()
		log_entries = []

		for line in Dir.yield_lines(experiment.file_path, ITEM_LIMIT):
			log_entry = LogEntry.from_log_string(line)
			log_entries.append(log_entry)

		all_entries = converter.LOG_ENTRIES_TO_IDS_ENTRIES(log_entries, binary=True)

		training_entries, scoring_entries = ids_tools.ids_entries_to_train_test(all_entries)
		X_train, _ = IdsConverter().ids_entries_to_X_y(training_entries)

		scoring_dict = {}
		for ids_entry in scoring_entries:
			if ids_entry.app_id not in scoring_dict:
				scoring_dict[ids_entry.app_id] = []
			scoring_dict[ids_entry.app_id].append(ids_entry)

		# Classify with all entries: training_entries
		classifiers = [
			sk_svm.OneClassSVM(),
			sk_ens.IsolationForest()
		]
		for classifier in classifiers:
			classifier.fit(X_train)

		# Score for each app: scoring_dict
		for app_id, app_entries in util.seqr.yield_items_in_key_order(scoring_dict):
			X_test, y_true = IdsConverter().ids_entries_to_X_y(app_entries)
			y_preds = [clf.predict(X_test) for clf in classifiers]
			for clf, y_pred in zip(classifiers, y_preds):
				experiment.visualise_store("ALL", app_id, clf, y_true, y_pred)


	@staticmethod
	def handle_app(app_id, ids_entries, experiment):
		""" Full flow for one classifier. """

		verify_ids_entries(ids_entries, app_id, experiment.storer_printer)

		training, scoring = ids_tools.ids_entries_to_train_test(ids_entries)
		X_train, _ = IdsConverter().ids_entries_to_X_y(training)
		X_test, y_true = IdsConverter().ids_entries_to_X_y(scoring)

		classifiers = [
			sk_svm.OneClassSVM(),
			sk_ens.IsolationForest()
		]
		for classifier in classifiers:
			classifier.fit(X_train)
			y_pred = classifier.predict(X_test)
			experiment.visualise_store("SPEC", app_id, classifier, y_true, y_pred)


# _, _ = self.preprocess_fit_score(app_id, ids_entries,
# 	lambda x: x,
# 	sklearn.svm.OneClassSVM(),
# 	printer)

# name = "IF"
# n_est = 100
# max_sampl = 256
# self.storer_printer.prt("\n\t> %s - n_est: %s, max_sampl: %s" % (name, n_est, max_sampl))
# _, _ = self.preprocess_fit_score(app_id, ids_entries,
# 	lambda x: x,
# 	sk_ens.IsolationForest(n_estimators=n_est, max_samples=max_sampl, n_jobs=-1, random_state=0),
# 	printer)

# _, _ = self.preprocess_fit_score(app_id, ids_entries,
# 	lambda x: sk_pre.scale(x),
# 	sklearn.svm.OneClassSVM(),
# 	printer)


# def preprocess_fit_score(self, app_id, ids_entries, preprocessor, classifier):
# 	""" Use the given preprocessor on the data, classify it with the given classifier and score. """

# 	converter = IdsConverter()
# 	X, y = converter.ids_entries_to_X_y(ids_entries, app_id)

# 	self.storer_printer.prt("Preprocessing... ", newline=False)
# 	X = preprocessor(X)

# 	self.storer_printer.prt("Splitting... ", newline=False)
# 	X_train, _, X_test, y_true = ids_tools.X_y_to_train_test(X, y)

# 	self.storer_printer.prt("Fitting... ", newline=False)
# 	classifier.fit(X_train)

# 	self.storer_printer.prt("Predicting... ")
# 	y_pred = classifier.predict(X_test)

# 	self.visualise_store(app_id, app_id, classifier, y_true, y_pred)

# 	return (y_true, y_pred)
