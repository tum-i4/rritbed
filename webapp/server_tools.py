#!/usr/bin/env python
""" Tools for command-line interaction with the server """

import argparse
import os
import statistics as stat
import sys
import time
import warnings

import sklearn
import sklearn.metrics as sk_met
import sklearn.model_selection as sk_mod

from log_entry import LogEntry
from state_dao import StateDao
import log_file_analysis
import util.fmtr
import util.outp
import util.prtr
import util.seqr
import util.stat
from ids.dir_utils import Dir, ModelDir
from ids.intrusion_classifier import IntrusionClassifier
from ids.ids_converter import IdsConverter
import ids.ids_tools as ids_tools
import ids.ids_data as ids_data


_HISTORY_FILE = "intrusion_classifier_history"


def train_call(args):
	""" Unpack the args and call _train.
	Expects 'train_file_path'. """
	_train(args.train_file_path)


def _train(file_path):
	""" Train the classifier with the given file. """

	print("Using file \"{}\"".format(os.path.join(os.getcwd(), file_path)))

	saved_so_far = []

	if os.path.lexists(_HISTORY_FILE):
		saved_so_far = Dir.read_lines(_HISTORY_FILE)

	if file_path in saved_so_far:
		print("This file has already been used for training."
			+ " If you think this is a mistake, rename it and run again.")
		return

	log_entry_generator = _yield_log_entries_from_file(file_path)
	_train_entries(log_entry_generator)

	with open(_HISTORY_FILE, 'a') as hist_file:
		hist_file.write(file_path + "\n")


def _train_entries(log_entry_generator, squelch_output=False):
	"""
	Train with the given LogEntry objects.
	returns: Boolean flag indicating success
	"""

	clas = IntrusionClassifier.get_singleton()

	try:
		clas.train(log_entry_generator, squelch_output=squelch_output)
		return True
	except ValueError as val_err:
		print(val_err)
		return False


def score_call(args):
	""" Unpack the args and call _score.
	Expects 'test_file_path' and 'iterations'. """
	_score(args.test_file_path, args.iterations)


def _score(file_path, iterations):
	""" Score the given LogEntry objects in multiple iterations and print the output. """

	log_entries = _read_file_flow(file_path)

	scores = {}
	for app_id in ids_data.get_app_ids():
		scores[app_id] = []

	printer = util.prtr.Printer()

	printer.prt("Scoring in {} iterations: ".format(iterations), newline=False)

	for i in range(iterations, 0, -1):
		# Score
		printer.prt("{}...".format(i), newline=False)
		scoring_result = _score_entries(log_entries, squelch_output=True)
		if not scoring_result:
			printer.prt("")
			printer.prt("Scoring failed!")
			# Don't continue; reset needs to happen in order to allow for the next iteration

		for app_id in scoring_result:
			scores[app_id].append(scoring_result[app_id])

	printer.prt("Done!")

	_print_scores(scores, printer)


def _score_entries(log_entries, squelch_output=False):
	"""
	Score the given LogEntry objects.
	returns: Boolean flag indicating success
	"""

	clas = IntrusionClassifier.get_singleton()

	try:
		result = clas.score(log_entries, do_return=True, squelch_output=squelch_output)
		return result
	except ValueError as val_err:
		print(val_err)
		return None


def train_score_call(args):
	""" Unpack the args and call _train_and_score.
	Expects 'file_path', 'folds' and optionally 'iterations'. """
	_train_and_score(args.file_path, args.folds, args.iterations)


def _train_and_score(file_path, folds, iterations=None):
	"""
	Use the given log_entries to score the classifier in a <fold>-fold cross-validation.
	: param iterations : Optionally specify to repeat <iterations> times.
	"""

	if iterations <= 1:
		iterations = None

	log_entries = _read_file_flow(file_path)

	if len(log_entries) < 10000:
		raise IOError("Insufficient number of entries found in the file. Need >= 10,000.")

	scores = {}
	for app_id in ids_data.get_app_ids():
		scores[app_id] = []

	printer = util.prtr.Printer()

	printer.prt("Using {}-fold cross-validation".format(folds)
		+ "" if iterations is None else " with {} iteration{}."
			.format(iterations, "s" if iterations > 1 else ""))

	folds = None
	if iterations:
		folds = sk_mod.RepeatedKFold(n_splits=folds, n_repeats=iterations)
	else:
		folds = sk_mod.KFold(n_splits=folds)

	current_round = 1

	for train_indices, score_indices in folds.split(log_entries):
		printer.prt("Round {} of {}.".format(current_round, folds * iterations))
		current_round += 1

		# Selecting items based on the given indices
		printer.prt("Splitting... ", newline=False)
		training_entries = [log_entries[i] for i in train_indices]
		scoring_entries = [log_entries[i] for i in score_indices]

		preconditions_msg = "Please make sure that all preconditions are met and rerun."

		# Train
		printer.prt("Training... ", newline=False)
		training_succeeded = _train_entries(training_entries, squelch_output=True)
		if not training_succeeded:
			printer.prt("")
			printer.prt("Training failed. " + preconditions_msg)
			continue

		# Score
		printer.prt("Scoring... ", newline=False)
		scoring_result = _score_entries(scoring_entries, squelch_output=True)
		if not scoring_result:
			printer.prt("")
			printer.prt("Scoring failed. " + preconditions_msg)
			# Don't continue; reset needs to happen in order to allow for the next iteration

		for app_id in scoring_result:
			scores[app_id].append(scoring_result[app_id])

		# Reset
		printer.prt("Resetting... ", newline=False)
		IntrusionClassifier.reset_models(purge=True)
		printer.prt("Done.")

	_print_scores(scores, printer)


def score_pr_call(args):
	""" Unpack the args and call _score_pr.
	Expects 'file_path'. """
	_score_pr(args.file_path)


def _score_pr(file_path):

	printer = util.prtr.Printer()
	squelcher = util.prtr.Printer(squelch=True)
	converter = IdsConverter()

	log_entries = _read_file_flow(file_path)

	scores_acc = _empty_app_id_dict()
	scores_prec = _empty_app_id_dict()
	scores_rec = _empty_app_id_dict()

	printer.prt("Preparing... ", newline=False)

	# converted_entries: [(app_id, vector, class)]
	converted_entries = []
	for log_entry in log_entries:
		converted_entries.append(converter.log_entry_to_prepared_tuple(log_entry, binary=True))

	printer.prt("Filtering... ", newline=False)
	train_entries, test_entries = ids_tools.converted_entries_to_train_test(converted_entries)

	printer.prt("Splitting... ", newline=False)
	train_dict = converter.prepared_tuples_to_train_dict(train_entries, squelcher)
	test_dict = converter.prepared_tuples_to_train_dict(test_entries, squelcher)

	result_table = []
	result_table.append(["App id", "Actual (+)", "Actual (-)"])
	printer.prt("Scoring... ")
	for app_id in converter.app_ids:

		X_train, y_train = train_dict[app_id]
		X_test, y_test = test_dict[app_id]

		clf = sklearn.svm.OneClassSVM(random_state=0)
		clf.fit(X_train)

		result = clf.predict(X_test)

		# TODO MOAR
		warnings.filterwarnings("ignore", category=sklearn.exceptions.UndefinedMetricWarning)
		scores_acc[app_id].append(sk_met.accuracy_score(y_test, result))
		scores_prec[app_id].append(sk_met.precision_score(y_test, result))
		scores_rec[app_id].append(sk_met.recall_score(y_test, result))

		tn, fp, fn, tp = sk_met.confusion_matrix(y_test, result).ravel()
		result_table.append(["{} (+)".format(app_id), tp, fp])
		result_table.append(["{} (-)".format(" " * len(app_id)), fn, tn])
		dash = "-" * 10
		result_table.append([dash, dash, dash])

	_print_scores(scores_acc, printer, headline="Accuracy")
	_print_scores(scores_prec, printer, headline="Precision")
	_print_scores(scores_rec, printer, headline="Recall")
	util.outp.print_table(result_table, headline="Confusion matrix", printer=printer)


def _print_scores(scores, printer, headline="Results"):
	""" Print the given scores in a table. """

	if not scores or not scores.itervalues().next():
		printer.prt("No results!")
		return

	result_table = []

	if len(scores.itervalues().next()) == 1:
		result_table.append(["Classifier", "Score"])
		for app_id in scores:
			result_table.append([app_id, util.fmtr.format_percentage(scores[app_id][0], True)])
	else:
		result_table.append(["App id", "Avg. score", "Variance", "", "All scores"])
		for app_id in scores:
			row = scores[app_id]
			result_table.append([
				app_id,
				util.fmtr.format_percentage(util.stat.avg(row)),
				round(stat.variance([x * 100 for x in row]), 2),
				"",
				", ".join([util.fmtr.format_percentage(x, pad_spaces=True) for x in row])
			])

	util.outp.print_table(result_table, headline=headline, printer=printer)


def reset_call(args):
	""" Call _reset. Expects 'classifier', 'server_log' or 'all'. """
	_reset(args.classifier, args.server_log, args.all)


def _reset(classifier, server_log, reset_all):
	""" Move the generated models to a sub-folder and reset the intrusion_classifier_history. """

	if not (classifier or server_log or reset_all):
		print("No selection was made! Add either --classifier, --server-log or --all to reset.")
		return

	classifier |= reset_all
	server_log |= reset_all

	print("Resetting {}{}{}...".format(
		"classifier" if classifier else "",
		" and " if classifier and server_log else "",
		"server log" if server_log else ""
	))

	message = ""

	if classifier:
		message += "\n# Classifier\nTraining history: "
		if os.path.lexists(_HISTORY_FILE):
			os.remove(_HISTORY_FILE)
			message += "File was removed."
		else:
			message += "No history file found."

		message += "\nModel directory: " + ModelDir.reset_dir()
		if server_log:
			message += "\n"

	if server_log:
		message += "\n# Server log\n"
		message += StateDao.reset_in_instance()

	print(message)


def _read_file_flow(file_path, squelch_output=False):
	""" Read up to 5000000 lines of the given file as LogEntry objects.
	Updates the user about the progress. """

	printer = util.prtr.Printer(squelch=squelch_output)

	log_entries = []

	printer.prt("Using log file \"{}\"".format(os.path.join(os.getcwd(), file_path)))
	printer.prt("Reading file and converting up to 5,000,000 lines to LogEntry objects...")
	log_entries = _get_log_entries_from_file(file_path, 5000000)

	printer.prt("Done.")
	return log_entries


def _get_log_entries_from_file(file_path, limit):
	""" Read up to <limit> number of log entries from the given file. """

	log_entries = []

	for line in Dir.yield_lines(file_path, limit):
		log_entries.append(LogEntry.from_log_string(line))

	return log_entries


def _yield_log_entries_from_file(file_path):
	for line in Dir.yield_lines(file_path):
		yield LogEntry.from_log_string(line)



if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		SUBPARSERS = PARSER.add_subparsers()

		TRAIN_PARSER = SUBPARSERS.add_parser("train", help="Train the classifier")
		TRAIN_PARSER.add_argument("train_file_path", metavar="PATH", help="The training data")
		TRAIN_PARSER.set_defaults(function=train_call)

		SCORE_PARSER = SUBPARSERS.add_parser("score", help="Score the predictions of the current models")
		SCORE_PARSER.add_argument("test_file_path", metavar="PATH", help="The test data")
		SCORE_PARSER.add_argument("--iterations", "-i", type=int, default=1)
		SCORE_PARSER.set_defaults(function=score_call)

		TRAINSCORE_PARSER = SUBPARSERS.add_parser("train-and-score", help="Split, train, score, reset")
		TRAINSCORE_PARSER.add_argument("file_path", metavar="PATH", help="The data")
		TRAINSCORE_PARSER.add_argument("--folds", "-f", type=int, default=5)
		TRAINSCORE_PARSER.add_argument("--iterations", "-i", type=int)
		TRAINSCORE_PARSER.set_defaults(function=train_score_call)

		SCOREPR_PARSER = SUBPARSERS.add_parser("score-pr")
		SCOREPR_PARSER.add_argument("file_path", metavar="PATH", help="The data")
		SCOREPR_PARSER.set_defaults(function=score_pr_call)

		RESET_PARSER = SUBPARSERS.add_parser("reset", help="Reset the classifier")
		RESET_PARSER.add_argument("--classifier", "-c", action="store_true")
		RESET_PARSER.add_argument("--server-log", "-l", action="store_true")
		RESET_PARSER.add_argument("--all", "-a", action="store_true")
		RESET_PARSER.set_defaults(function=reset_call)

		if len(sys.argv) == 1:
			PARSER.print_help()
			exit()

		ARGS = PARSER.parse_args()

		START_TIME = time.time()
		# Actual functionality
		ARGS.function(ARGS)
		TIME_EXPIRED = time.time() - START_TIME
		print("")
		util.outp.print_time_passed_message(TIME_EXPIRED, sys.argv[1])
		exit()
	except KeyboardInterrupt:
		pass
