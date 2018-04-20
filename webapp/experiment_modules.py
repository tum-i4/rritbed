#!/usr/bin/env python
""" Runnable experiments. All respect the ModuleInterface interface. """

# pylint: disable-msg=R0903, C0103; (Too few public methods, invalid name)

import time

import sklearn.ensemble as sk_ens
import sklearn.svm as sk_svm

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
			"Benchmark result | %s entries processed | OneClassSVM classifier",
			"",
			"Mapping: %s" % util.fmtr.format_time_passed(time_for_map),
			"One-hot: %s" % util.fmtr.format_time_passed(time_for_one_hot)
		]

		experiment.add_result_file("time_map_vs_onehot", timing_lines)


	@staticmethod
	def handle_log_entries(name, converter, log_entries, experiment):

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


### Experiments ###


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
