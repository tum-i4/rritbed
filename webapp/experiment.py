#!/usr/bin/env python
""" The Experiment class and program. """

# pylint: disable-msg=C0103; (Invalid name)

import argparse
import os
import random
import time
from collections import namedtuple

import sklearn
import sklearn.ensemble as sk_ens
import sklearn.externals as sk_ext
import sklearn.metrics as sk_metr
import sklearn.preprocessing as sk_pre

from log_entry import LogEntry
import ids.ids_data as ids_data
import ids.ids_tools as ids_tools
from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter
from ids.ids_entry import IdsEntry
import idse_dao
import log_file_utils
import util.fmtr
import util.outp
from util.prtr import TimePrinter


ITEM_LIMIT = 5000000
EXPERIMENTS_HOME = "experiments"


ClassifierResultPair = namedtuple("ClassifierResultPair", "classifier result")


class Experiment(object):
	""" Do experiments. Start with a(). """

	def __init__(self, file_path, store_title):
		""" Ctor """

		object.__init__(self)

		# State
		self.title = None
		# Time
		self.start_time = time.time()
		self.end_time = None
		# Loaded entries
		self.entries_dict = {}
		self.entries_count = -1
		# ClassifierResultPair objects (classifier, result)
		self.classifier_results = []

		# Paths and name
		self.file_path = os.path.expanduser(file_path)
		self.input_file_name = os.path.basename(self.file_path)
		experiment_dir_name = None

		if not os.path.lexists(self.file_path):
			util.outp.exit_on_error("Log file not found: %s" % self.file_path)

		if store_title is not None:
			self.title = store_title
			experiment_dir_name = store_title.replace(" ", "_")
		else:
			date_str = time.strftime("%m-%d-%H-%M")
			random_num_str = "".join(str(x) for x in (random.sample(range(0, 15), 5)))
			self.title = "experiment_%s_%s" % (date_str, random_num_str)
			experiment_dir_name = self.title

		self.experiment_dir_path = self.get_experiment_folder(experiment_dir_name)

		if os.path.lexists(self.experiment_dir_path):
			util.outp.exit_on_error("Experiment folder exists: %s" % self.experiment_dir_path)


	### Workers ###


	def run(self):
		""" Read entries, convert them, split them per app_id and call b() for each app. """

		# Prerequisites: I could have loaded from a folder
		# raise NotImplementedError()

		printer = TimePrinter(name="experiment")
		printer.prt("Reading file and converting...")

		# ids_entries: { app_id, vector, my_class }
		self.entries_dict = self.read_convert(self.file_path)

		printer.prt("Done. Scoring each individual classifier...")

		# for app_id, ids_entries in self.entries_dict.items():
		# 	self.handle_app(app_id, ids_entries)

		printer.prt("Done. Scoring all classifiers...")

		# TODO TEMP?
		self.handle_all(self.file_path)

		self.store_experiment()


	def handle_all(self, file_path):
		""" Full flow for a one-fits-all classifier. """

		printer = util.prtr.TimePrinter(name="ALL")
		printer.prt("ALLLLLLLL\n\n")

		from ids.TEMP_IDS_CONVERTER import IdsConverter as TEMPCONVERTER
		converter = TEMPCONVERTER()
		log_entries = []

		for line in Dir.yield_lines(file_path, ITEM_LIMIT):
			log_entry = LogEntry.from_log_string(line)
			log_entries.append(log_entry)

		all_entries = converter.LOG_ENTRIES_TO_IDS_ENTRIES(log_entries, binary=True)

		training_entries, scoring_entries = ids_tools.ids_entries_to_train_test(all_entries)
		X_train, _ = IdsConverter.ids_entries_to_X_y(training_entries)

		scoring_dict = ids_tools.empty_app_id_to_list_dict()
		for ids_entry in scoring_entries:
			scoring_dict[ids_entry.app_id].append(ids_entry)

		# Classify with all entries: training_entries
		classifier = sklearn.svm.OneClassSVM()
		classifier.fit(X_train)

		# Score for each app: scoring_dict
		for app_id, app_entries in scoring_dict.items():
			X_test, y_true = IdsConverter.ids_entries_to_X_y(app_entries)
			y_pred = classifier.predict(X_test)
			self.visualise_store(app_id, classifier, y_true, y_pred)

		printer.prt("\n\n\DONNNNNNEEEE\n\n")


	def handle_app(self, app_id, ids_entries):
		""" Full flow for one classifier. """

		printer = TimePrinter(name=app_id)

		if not ids_entries:
			printer.prt("No input data for {}".format(app_id))
			return

		if not isinstance(ids_entries[0], IdsEntry):
			raise TypeError("Given list does not contain IdsEntry objects.")

		ids_entries = ids_tools.straighten_dataset_for_app(ids_entries)

		print("\n\t::: %s :::\n" % app_id)

		# TODO
		_, _ = self.preprocess_fit_score(app_id, ids_entries,
			lambda x: x,
			sklearn.svm.OneClassSVM(),
			printer)

		name = "IF"
		n_est = 100
		max_sampl = 256
		print("\n\t> %s - n_est: %s, max_sampl: %s" % (name, n_est, max_sampl))
		_, _ = self.preprocess_fit_score(app_id, ids_entries,
			lambda x: x,
			sk_ens.IsolationForest(n_estimators=n_est, max_samples=max_sampl, n_jobs=-1, random_state=0),
			printer)

		# _, _ = self.preprocess_fit_score(app_id, ids_entries,
		# 	lambda x: sk_pre.scale(x),
		# 	sklearn.svm.OneClassSVM(),
		# 	printer)


	def preprocess_fit_score(self, app_id, ids_entries, preprocessor, classifier, printer):
		""" Use the given preprocessor on the data, classify it with the given classifier and score. """

		converter = IdsConverter()
		X, y = converter.ids_entries_to_X_y(ids_entries, app_id)

		printer.prt("Preprocessing... ", newline=False)
		X = preprocessor(X)

		printer.prt("Splitting... ", newline=False)
		X_train, _, X_test, y_true = ids_tools.X_y_to_train_test(X, y)

		printer.prt("Fitting... ", newline=False)
		classifier.fit(X_train)

		printer.prt("Predicting... ")
		y_pred = classifier.predict(X_test)

		self.visualise_store(app_id, classifier, y_true, y_pred)

		return (y_true, y_pred)


	def visualise_store(self, app_id, classifier, y_true, y_pred):
		""" Score, print. """

		# Result: [accuracy, precision, recall, tn, fp, fn, tp, confusion_matrix]
		this_result = []

		print("\nSCORE FOR >>> %s <<<" % app_id)

		accu = sk_metr.accuracy_score(y_true, y_pred)
		prec = sk_metr.precision_score(y_true, y_pred)
		reca = sk_metr.recall_score(y_true, y_pred)

		this_result.extend([("Accuracy", accu), ("Precision", prec), ("Recall", reca)])

		print("PREC: %s, RECC: %s, ACCU: %s" % (prec, reca, accu))

		tn, fp, fn, tp = sk_metr.confusion_matrix(y_true, y_pred).ravel()

		this_result.extend([("TN", tn), ("FP", fp), ("FN", fn), ("TP", tp)])

		storer = util.prtr.Storer()

		table = []
		table.append(["", "Actual (+)", "Actual (-)"])
		table.append(["Pred (+)", tp, fp])
		table.append(["Pred (-)", fn, tn])
		util.outp.print_table(table, printer=storer)

		this_result.append(storer.get_messages())
		storer.printout()

		self.classifier_results.append(
			ClassifierResultPair(classifier=classifier, result=this_result)
		)

		print("\nEND FOR  >>> %s <<<" % app_id)


	def e(self, app_id, ids_entries):

		if app_id not in ids_data.get_generators():
			return

		import numpy as np
		from sklearn.covariance import EllipticEnvelope
		from sklearn.svm import OneClassSVM
		import matplotlib.pyplot as plt
		import matplotlib.font_manager

		X_list, y = IdsConverter().ids_entries_to_X_y(ids_entries, app_id)
		X = np.array(X_list)

		X_scaled = sk_pre.scale(X)

		print("WTF\n")


	### Persistence ###


	def store_experiment(self):

		printer = util.prtr.Printer(name="store")
		printer.prt("Storing experiment results...")

		self.open_experiment_folder()

		entry_file_path = os.path.join(self.experiment_dir_path, "used_entries")
		result_file_path = os.path.join(self.experiment_dir_path, "result")
		other_file_paths = [entry_file_path, result_file_path]
		classifiers_file_paths = [
			os.path.join(self.experiment_dir_path, type(x).__name__).replace(" ", "_")
			for (x, _) in self.classifier_results
		]

		if any([os.path.lexists(x) for x in other_file_paths + classifiers_file_paths]):
			raise IOError("One of the files exists: %s" % (other_file_paths + classifiers_file_paths))

		all_my_entries = reduce(lambda a, b: a.extend(b), self.entries_dict.values(), [])
		assert(self.entries_count == len(all_my_entries))

		printer.prt("Data verified. Storing utilised entries...")

		# Create new file with my entries
		saved_path = idse_dao.save_entries(entry_file_path, all_my_entries)

		printer.prt("Done. Analysing file...")

		# Analyse that file
		log_file_utils.analyse(saved_path, to_file=True, output_printer=util.prtr.Storer())

		printer.prt("Done. Saving classifiers...")

		# Save trained classifiers
		for (classifier, _), its_path in zip(self.classifier_results, classifiers_file_paths):
			sk_ext.joblib.dump(classifier, its_path)

		printer.prt("Done. Saving result digest...")

		# Save the result
		result_lines = self.create_result_lines()
		Dir.write_lines(result_file_path, result_lines)

		printer.prt("Done!")
		printer.prt("Experiment stored in: %s" % self.experiment_dir_path)


	def retrieve_experiment(self, experiment_dir):
		raise NotImplementedError()
		# return (test_set, score_set, classifier, result)


	def create_result_lines(self):
		""" Create a human readable digest of the experiment. """

		exp_date_str = time.strftime("%d.%m.%Y %H:%M", time.localtime(self.start_time))
		time_passed_str = util.fmtr.format_time_passed(self.end_time - self.start_time)

		heading = ("\"%s\" | experiment on %s | running time: %s"
			% (self.title, exp_date_str, time_passed_str))

		lines = [
			heading,
			"-" * len(heading),
			""
		]

		for classifier, result in self.classifier_results:
			classifier_result = self.create_classifier_result(classifier, result)
			lines.extend(classifier_result)

		return lines


	def create_classifier_result(self, classifier, result):
		# Result: [accuracy, precision, recall, tn, fp, fn, tp, confusion_matrix]

		lines = ["Classifier:"]
		lines.append(str(classifier))

		result_line_1 = ">>> Result | "
		for element in result[:-1]:
			result_line_1 += "%s: %s |" % element
		lines.append(result_line_1[:-2])

		lines.append(">>> Confusion matrix:")
		lines.extend(result[-1:])

		return lines


	@staticmethod
	def get_experiment_folder(name):
		""" Get the experiment folder of the given name. """

		result = os.path.join(EXPERIMENTS_HOME, name)
		return result


	def open_experiment_folder(self):
		""" Ensure the experiment folder exists. """

		if not os.path.lexists(self.experiment_dir_path):
			os.makedirs(self.experiment_dir_path)


	### Helpers ###


	def read_convert(self, file_path):
		""" Read log entries from the given file and convert the result. """

		converter = IdsConverter()
		log_entries = []

		for line in Dir.yield_lines(file_path, ITEM_LIMIT):
			log_entry = LogEntry.from_log_string(line)
			log_entries.append(log_entry)

		self.entries_count = len(log_entries)

		ids_entries_dict = converter.log_entries_to_ids_entries_dict(log_entries)
		return ids_entries_dict



if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		PARSER.add_argument("file_path", metavar="PATH/FILE", help="Log file")
		PARSER.add_argument("--store", "-s", metavar="TITLE", help="Experiment title")

		ARGS = PARSER.parse_args()

		experiment = Experiment(ARGS.file_path, ARGS.store)
		experiment.run()

		exit()
	except KeyboardInterrupt:
		pass
