#!/usr/bin/env python
""" The Experiment class and program. """

import argparse
import os
import random
import time
from collections import namedtuple

import sklearn.metrics as sk_metr

import experiment_modules
from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter
import idse_dao
import log_file_analysis
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
		self.entries = []
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


	### Running, verifying ###


	def run(self):
		""" Read entries, convert them, split them per app_id and call b() for each app. """

		# TODO Prerequisites: I could have loaded from a folder

		# experiment_modules.AllVsSpecSvmVsIso.run(self)
		experiment_modules.OneHotVsLabelling.run(self)
		self.ensure_valid_state()
		self.store_experiment()


	def ensure_valid_state(self):
		""" Verify that a run experiment module correctly set the internal state. """

		if not self.entries or not self.classifier_results:
			raise ValueError("Experiment module did not store any results.")


	### Persistence ###


	def store_experiment(self):
		""" Store the results saved in this class in our experiment directory. """

		self.end_time = time.time()
		self.storer_printer.prt("Storing experiment results...")

		Dir.ensure_folder_exists(self.experiment_dir_path)

		entry_file_path = os.path.join(self.experiment_dir_path, "used_entries")
		result_file_path = os.path.join(self.experiment_dir_path, "result")
		stdout_file_path = os.path.join(self.experiment_dir_path, "stdout")
		classifiers_file_path = os.path.join(self.experiment_dir_path, "classifiers")
		file_paths = [entry_file_path, result_file_path, stdout_file_path, classifiers_file_path]

		if any([os.path.lexists(x) for x in file_paths]):
			raise IOError("One of the files exists: %s" % (file_paths))

		# self.storer_printer.prt("Data verified. Storing utilised entries...")

		# Create new file with my entries
		# saved_path = idse_dao.save_entries(entry_file_path, self.entries)

		# self.storer_printer.prt("Done. Analysing file...")

		# Analyse that file
		# log_file_analysis.analyse(saved_path, to_file=True, output_printer=util.prtr.Storer())

		self.storer_printer.prt("Done. Saving classifiers...")

		# Save trained classifiers
		classifier_lines = self.create_classifier_lines()
		Dir.write_lines(classifiers_file_path, classifier_lines)

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


	def create_classifier_lines(self):
		""" Create an overview of the utilised classifiers. """

		lines = ["Name | Classifier"]
		for name, classifier, _ in self.classifier_results:
			lines.append("%s | %s"
				% (util.fmtr.fit_string_in(name, 4), self.classifier_str_oneline(classifier)))

		return lines


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


	### Interface for experiment modules ###


	def visualise_store(self, name, app_id, classifier, y_true, y_pred):
		""" Score, print. """

		self.storer_printer.prt("Finished [%s] for app_id [%s] and classifier [%s]"
			% (name, app_id, self.classifier_name(classifier)))

		accu = sk_metr.accuracy_score(y_true, y_pred)
		prec = sk_metr.precision_score(y_true, y_pred)
		reca = sk_metr.recall_score(y_true, y_pred)

		# pylint: disable-msg=C0103; (Invalid name)
		tn, fp, fn, tp = sk_metr.confusion_matrix(y_true, y_pred).ravel()

		storer = util.prtr.Storer()

		table = []
		table.append(["", "Actual (+)", "Actual (-)"])
		table.append(["Pred (+)", tp, fp])
		table.append(["Pred (-)", fn, tn])
		util.outp.print_table(table, printer=storer)

		classifier_name = self.classifier_name(classifier)
		line_prefix = ">>> [%s / %s] %s -" % (
			util.fmtr.fit_string_in(name, 4),
			util.fmtr.fit_string_in(classifier_name, 20),
			util.fmtr.fit_string_in(app_id, 12))

		justed_value_str = lambda x: (util.fmtr.fit_string_in("{:.12f}".format(x), 15)).replace(".", ",")

		# Result: [app_id, accuracy, precision, recall, tn, fp, fn, tp, confusion_matrix]
		this_result = [
			"Classifier: %s (%s) | %s" % (name, classifier_name, self.classifier_str_oneline(classifier)),
			"",
			("%s Result | Accuracy: %s | Precision: %s | Recall: %s"
				% (line_prefix, justed_value_str(accu), justed_value_str(prec), justed_value_str(reca))),
			"%s Confusion matrix:" % line_prefix
		]

		table_lines = [line for line in storer.get_messages() if line != ""]

		this_result.extend(table_lines)

		self.storer_printer.prt("> Precision: %s | Recall: %s | Accuracy: %s" % (prec, reca, accu))

		self.classifier_results.append(
			ClassifierResultGroup(name=name, classifier=classifier, result=this_result)
		)


	def read_convert(self, file_path):
		""" Read IDS entries from the given file and convert the result. """

		converter = IdsConverter()

		self.entries = list(idse_dao.yield_entries(file_path))
		ids_entries_dict = converter.ids_entries_to_dict(self.entries)

		return ids_entries_dict


	### Helpers ###


	@staticmethod
	def classifier_name(classifier):
		""" Return a human-readable name for the given classifier. """
		return type(classifier).__name__


	@staticmethod
	def classifier_str_oneline(classifier):
		""" Return a one-line description for the given classifier. """
		return str(classifier).replace("\n", "").replace("      ", " ")



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
