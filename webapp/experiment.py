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

from log_entry import LogEntry
import ids.ids_tools as ids_tools
from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter
from ids.ids_entry import IdsEntry
import idse_dao
import log_file_utils
import util.fmtr
import util.outp
import util.prtr
import util.seqr


ITEM_LIMIT = 5000000
EXPERIMENTS_HOME = "experiments"


ClassifierResultGroup = namedtuple("ClassifierResultGroup", "name classifier result")


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
		self.ids_entries = []
		# ClassifierResultGroup objects (name, classifier, result)
		self.classifier_results = []

		# StorerAndPrinter - stores and prints ;)
		time_printer = util.prtr.TimePrinter(name="exp")
		self.storer_printer = util.prtr.StorerAndPrinter(printer=time_printer)

		# Paths and name
		self.file_path = os.path.expanduser(file_path)
		self.input_file_name = os.path.basename(self.file_path)
		experiment_dir_name = None

		if not os.path.lexists(self.file_path):
			util.outp.exit_on_error("Input file not found: %s" % self.file_path)

		self.title = store_title
		if self.title is None:
			random_num_str = "".join(str(x) for x in (random.sample(range(0, 15), 5)))
			self.title = "Experiment %s" % random_num_str

		experiment_dir_name = Dir.remove_disallowed_characters(self.title.lower())
		experiment_dir_name += time.strftime("_%m-%d_%H-%M")
		self.experiment_dir_path = self.get_experiment_folder(experiment_dir_name)

		if os.path.lexists(self.experiment_dir_path):
			self.experiment_dir_path = Dir.uniquify(self.experiment_dir_path)


	### Workers ###


	def run(self):
		""" Read entries, convert them, split them per app_id and call b() for each app. """

		# Prerequisites: I could have loaded from a folder
		# raise NotImplementedError()

		# TODO TEMP?
		self.storer_printer.prt("Scoring classifier trained with all samples...")
		self.handle_all(self.file_path)

		self.storer_printer.prt("Done. Reading file and converting...")
		# ids_entries: { app_id, vector, my_class }
		ids_entries_dict = self.read_convert(self.file_path)

		self.storer_printer.prt("Done. Scoring individual classifier...")
		for app_id, ids_entries in util.seqr.yield_items_in_key_order(ids_entries_dict):
			self.handle_app(app_id, ids_entries)

		self.store_experiment()


	def handle_all(self, file_path):
		""" Full flow for a one-fits-all classifier. """

		self.storer_printer.prt("ALLLLLLLL\n\n")

		from ids.TEMP_IDS_CONVERTER import IdsConverter as TEMPCONVERTER
		converter = TEMPCONVERTER()
		log_entries = []

		for line in Dir.yield_lines(file_path, ITEM_LIMIT):
			log_entry = LogEntry.from_log_string(line)
			log_entries.append(log_entry)

		all_entries = converter.LOG_ENTRIES_TO_IDS_ENTRIES(log_entries, binary=True)

		training_entries, scoring_entries = ids_tools.ids_entries_to_train_test(all_entries)
		X_train, _ = IdsConverter.ids_entries_to_X_y(training_entries)

		scoring_dict = {}
		for ids_entry in scoring_entries:
			if ids_entry.app_id not in scoring_dict:
				scoring_dict[ids_entry.app_id] = []
			scoring_dict[ids_entry.app_id].append(ids_entry)

		# Classify with all entries: training_entries
		classifiers = [
			sklearn.svm.OneClassSVM(),
			sk_ens.IsolationForest()
			# sk_nghb.LocalOutlierFactor()
		]
		for classifier in classifiers:
			classifier.fit(X_train)

		# Score for each app: scoring_dict
		for app_id, app_entries in util.seqr.yield_items_in_key_order(scoring_dict):
			X_test, y_true = IdsConverter.ids_entries_to_X_y(app_entries)
			y_preds = [clf.predict(X_test) for clf in classifiers]
			for clf, y_pred in zip(classifiers, y_preds):
				self.visualise_store("ALL", app_id, clf, y_true, y_pred)

		self.storer_printer.prt("\n\nDONNNNNNEEEE\n\n")


	def handle_app(self, app_id, ids_entries):
		""" Full flow for one classifier. """

		if not ids_entries:
			self.storer_printer.prt("No input data for {}".format(app_id))
			return

		if not isinstance(ids_entries[0], IdsEntry):
			raise TypeError("Given list does not contain IdsEntry objects.")

		# ids_entries = ids_tools.straighten_dataset_for_app(ids_entries)

		# TODO TEMP FOR ONE-V-ALL
		training, scoring = ids_tools.ids_entries_to_train_test(ids_entries)
		X_train, _ = IdsConverter.ids_entries_to_X_y(training)
		X_test, y_true = IdsConverter.ids_entries_to_X_y(scoring)

		classifiers = [
			sklearn.svm.OneClassSVM(),
			sk_ens.IsolationForest()
			# sk_nghb.LocalOutlierFactor()
		]
		for classifier in classifiers:
			classifier.fit(X_train)
			y_pred = classifier.predict(X_test)
			self.visualise_store("SPEC", app_id, classifier, y_true, y_pred)

		# END TODO

		# self.storer_printer.prt("\n\t::: %s :::\n" % app_id)

		# # TODO
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


	def preprocess_fit_score(self, app_id, ids_entries, preprocessor, classifier):
		""" Use the given preprocessor on the data, classify it with the given classifier and score. """

		converter = IdsConverter()
		X, y = converter.ids_entries_to_X_y(ids_entries, app_id)

		self.storer_printer.prt("Preprocessing... ", newline=False)
		X = preprocessor(X)

		self.storer_printer.prt("Splitting... ", newline=False)
		X_train, _, X_test, y_true = ids_tools.X_y_to_train_test(X, y)

		self.storer_printer.prt("Fitting... ", newline=False)
		classifier.fit(X_train)

		self.storer_printer.prt("Predicting... ")
		y_pred = classifier.predict(X_test)

		self.visualise_store(app_id, app_id, classifier, y_true, y_pred)

		return (y_true, y_pred)


	def visualise_store(self, name, app_id, classifier, y_true, y_pred):
		""" Score, print. """

		self.storer_printer.prt("\nSCORE FOR >>> %s <<<" % app_id)

		accu = sk_metr.accuracy_score(y_true, y_pred)
		prec = sk_metr.precision_score(y_true, y_pred)
		reca = sk_metr.recall_score(y_true, y_pred)

		tn, fp, fn, tp = sk_metr.confusion_matrix(y_true, y_pred).ravel()

		storer = util.prtr.Storer()

		table = []
		table.append(["", "Actual (+)", "Actual (-)"])
		table.append(["Pred (+)", tp, fp])
		table.append(["Pred (-)", fn, tn])
		util.outp.print_table(table, printer=storer)

		classifier_name = type(classifier).__name__
		line_prefix = ">>> [%s / %s] %s -" % (
			util.fmtr.fit_string_in(name, 4),
			util.fmtr.fit_string_in(classifier_name, 20),
			util.fmtr.fit_string_in(app_id, 12))

		justed_value_str = lambda x: (util.fmtr.fit_string_in("{:.12f}".format(x), 15)).replace(".", ",")

		# Result: [app_id, accuracy, precision, recall, tn, fp, fn, tp, confusion_matrix]
		this_result = [
			"Classifier: %s (%s)" % (name, classifier_name),
			str(classifier),
			"",
			("%s Result | Accuracy: %s | Precision: %s | Recall: %s"
				% (line_prefix, justed_value_str(accu), justed_value_str(prec), justed_value_str(reca))),
			"%s Confusion matrix:" % line_prefix
		]

		table_lines = [line for line in storer.get_messages() if line != ""]

		this_result.extend(table_lines)

		self.storer_printer.prt("PREC: %s, RECC: %s, ACCU: %s" % (prec, reca, accu))
		storer.printout(purge=True)

		self.classifier_results.append(
			ClassifierResultGroup(name=name, classifier=classifier, result=this_result)
		)

		self.storer_printer.prt("\nEND FOR  >>> %s <<<" % app_id)


	### Persistence ###


	def store_experiment(self):
		""" Store the results saved in this class in our experiment directory. """

		self.end_time = time.time()
		self.storer_printer.prt("Storing experiment results...")

		Dir.ensure_folder_exists(self.experiment_dir_path)

		entry_file_path = os.path.join(self.experiment_dir_path, "used_entries")
		result_file_path = os.path.join(self.experiment_dir_path, "result")
		stdout_file_path = os.path.join(self.experiment_dir_path, "stdout")
		other_file_paths = [entry_file_path, result_file_path, stdout_file_path]
		classifiers_file_paths = []
		for name, classifier, _ in self.classifier_results:
			clf_name = "%s_%s" % (name, type(classifier).__name__.replace(" ", "_"))
			clf_path = os.path.join(self.experiment_dir_path, clf_name)
			while clf_path in classifiers_file_paths:
				clf_path += "_"
			classifiers_file_paths.append(clf_path)

		if any([os.path.lexists(x) for x in other_file_paths + classifiers_file_paths]):
			raise IOError("One of the files exists: %s" % (other_file_paths + classifiers_file_paths))

		self.storer_printer.prt("Data verified. Storing utilised entries...")

		# Create new file with my entries
		saved_path = idse_dao.save_entries(entry_file_path, self.ids_entries)

		self.storer_printer.prt("Done. Analysing file...")

		# Analyse that file
		log_file_utils.analyse(saved_path, to_file=True, output_printer=util.prtr.Storer())

		self.storer_printer.prt("Done. Saving classifiers...")

		# Save trained classifiers
		for (_, classifier, _), its_path in zip(self.classifier_results, classifiers_file_paths):
			sk_ext.joblib.dump(classifier, its_path)

		self.storer_printer.prt("Done. Saving result digest...")

		# Save the result
		result_lines = self.create_result_lines()
		Dir.write_lines(result_file_path, result_lines)

		self.storer_printer.prt("Done!")
		self.storer_printer.prt("Experiment stored in: %s" % self.experiment_dir_path)

		# Save the stdout (tee replacement)
		stdout_lines = self.storer_printer.get_messages()
		Dir.write_lines(stdout_file_path, stdout_lines)


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

		for _, _, result in self.classifier_results:
			lines.extend(result)
			lines.extend(["", ""])

		return lines


	@staticmethod
	def get_experiment_folder(name):
		""" Get the experiment folder of the given name. """

		result = os.path.join(EXPERIMENTS_HOME, name)
		return result


	### Helpers ###


	def read_convert(self, file_path):
		""" Read IDS entries from the given file and convert the result. """

		converter = IdsConverter()

		self.ids_entries = list(idse_dao.yield_entries(file_path))
		ids_entries_dict = converter.ids_entries_to_dict(self.ids_entries)

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
