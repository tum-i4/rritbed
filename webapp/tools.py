#!/usr/bin/env python
""" Tools for command-line interaction with the server """

import argparse
import cPickle
import os
import time

from log_entry import LogEntry
from ids.intrusion_classifier import IntrusionClassifier
from ids.dir_utils import Dir
import ids.ids_tools as ids_tools
import ids.ids_data as ids_data


_PICKLE_SUFFIX = ".pickle"
_HISTORY_FILE = "intrusion_classifier_history"


def train_call(args):
	""" Unpack the args and call _train.
	Expects 'train_file_path', 'multi_class' and 'extend_models'. """
	_train(args.train_file_path, args.multi_class, args.extend_models)


def _train(file_path, multi_class, extend_models):
	""" Train the classifier with the given file. Optionally allow extension of models. """
	saved_so_far = []

	if os.path.lexists(_HISTORY_FILE):
		saved_so_far = Dir.read_lines(_HISTORY_FILE)

	if file_path in saved_so_far:
		print("This file has already been used for training."
			+ " If you think this is a mistake, rename it and run again.")
		return

	log_entries = _read_file_flow(file_path)
	_train_entries(log_entries, multi_class, extend_models)

	with open(_HISTORY_FILE, 'a') as hist_file:
		hist_file.write(file_path + "\n")


def _train_entries(log_entries, multi_class, extend_models):
	"""
	Train with the given LogEntry objects.
	returns: Boolean flag indicating success
	"""

	clas = IntrusionClassifier.get_singleton()

	try:
		clas.train(log_entries, multi_class=multi_class, extend_models=extend_models)
		return True
	except ValueError as val_err:
		print(val_err.message)
		return False


def score_call(args):
	""" Unpack the args and call _score.
	Expects 'test_file_path' and 'multi_class'. """
	_score(args.test_file_path, args.multi_class)


def _score(file_path, multi_class):
	""" Score the prediction of the classifier with the given test file. """

	log_entries = _read_file_flow(file_path)
	_score_entries(log_entries, multi_class)


def _score_entries(log_entries, multi_class):
	"""
	Score the given LogEntry objects.
	returns: Boolean flag indicating success
	"""

	clas = IntrusionClassifier.get_singleton()

	try:
		clas.score(log_entries, multi_class=multi_class)
		return True
	except ValueError as val_err:
		print(val_err.message)
		return False


def train_score_call(args):
	""" Unpack the args and call _train_and_score.
	Expects 'file_path', 'split' and 'multi_class'. """
	_train_and_score(args.file_path, args.split, args.multi_class)


def _train_and_score(file_path, split, multi_class):
	""" Split the given file and use the first part for training, the second for scoring. """

	if split <= 0 or split >= 100:
		raise ValueError("Invalid split \"{}\" given.".format(split))

	log_entries = _read_file_flow(file_path)

	if len(log_entries) < 10000:
		raise IOError("Insufficient number of entries found in the file. Need >= 10,000.")

	training_entries, scoring_entries = _split_log_entries_flow(log_entries, split)

	preconditions_msg = "Please make sure that all preconditions are met and rerun."

	training_succeeded = _train_entries(training_entries, multi_class, extend_models=False)
	if not training_succeeded:
		print("Training failed. " + preconditions_msg)
		return

	scoring_succeeded = _score_entries(scoring_entries, multi_class)
	if not scoring_succeeded:
		print("Scoring failed. " + preconditions_msg)
		return


def convert_call(args):
	""" Unpack the args and call _convert.
	Expects 'file_path' and 'split' or 'pickle'. """
	if args.pickle:
		_convert_pickle(args.file_path)
	elif args.split:
		_convert_split(args.file_path, args.split)
	else:
		raise NotImplementedError("Arg configuration not implmented")


def _convert_split(file_path, split):
	""" Split the given file and pickle the results. """

	if split <= 0 or split >= 100:
		raise ValueError("Invalid split \"{}\" given.".format(split))

	log_entries = _read_file_flow(file_path)

	if len(log_entries) < 10000:
		print("Warning: Only {} entries found. Number might be too small for training or scoring"
			.format(len(log_entries)))

	training_entries, scoring_entries = _split_log_entries_flow(log_entries, split)

	training_file_path = file_path + "_train"
	scoring_file_path = file_path + "_score"

	try:
		_pickle_entries_flow(training_entries, training_file_path)
		_pickle_entries_flow(scoring_entries, scoring_file_path)
	except IOError as io_err:
		print(io_err.message)
		return

	print("Split was finished successfully!")
	return


def _convert_pickle(file_path):
	""" Pickle the given file. """

	log_entries = _read_file_flow(file_path)

	try:
		_pickle_entries_flow(log_entries, file_path)
	except IOError as io_err:
		print(io_err.message)
		return

	print("Pickling finished successfully!")


def reset_call(args):
	""" Call _reset. Expects nothing. """
	_reset()


def _reset():
	""" Move the generated models to a sub-folder and reset the intrusion_classifier_history. """
	raise NotImplementedError()


def anal_call(args):
	""" Unpack the args and call _analyse.
	Expects 'file_path'. """
	_analyse(args.file_path)


def _analyse(file_path):
	""" Analyse the given log file. """
	print("Analysing...")

	# Reading file #

	log_entries = _read_file_flow(file_path)

	# Analysis #

	all_app_ids = ids_data.get_app_ids()
	all_classes = ids_data.get_labels()

	(
		found_app_ids, entries_per_app_id, elements_per_class_per_app_id,
		found_classes, entries_per_class, app_ids_per_class
	) = _analyse_entries(log_entries)

	# Output #

	get_pl = lambda s, obj: s if len(obj) > 1 else ""

	print("")
	print("Found {} entries with {}/{} app id{} and {}/{} class{}".format(
		len(log_entries),
		len(found_app_ids), len(all_app_ids), get_pl("s", found_app_ids),
		len(found_classes), len(all_classes), get_pl("es", found_classes))
	)

	# App ID table

	per_app_id = []
	per_app_id.append(["App ID", "Elements"] + all_classes)
	for app_id in all_app_ids:
		line = [app_id, str(len(entries_per_app_id[app_id]))]

		for a_class in all_classes:
			class_count_str = ""
			if a_class in elements_per_class_per_app_id[app_id]:
				class_count_str = str(elements_per_class_per_app_id[app_id][a_class])
			line.append(class_count_str)

		per_app_id.append(line)

	_print_table(per_app_id, headline="Elements and classes per app ID")

	# Class table

	per_class = []
	per_class.append(["Class", "Elements", "App Ids"])
	for a_class in all_classes:
		per_class.append([a_class, len(entries_per_class[a_class]), len(app_ids_per_class[a_class])])

	_print_table(per_class, headline="Elements per class")

	# TODO: score??
	# harmonious? all labelled / some / none?
	# for each app id: are there roughly the same number of entries per class?
	return


def _analyse_entries(log_entries):
	"""
	Analyse the given LogEntry object.
	returns: A tuple containing (found_app_ids, entries_per_app_id, elements_per_class_per_app_id,
	found_classes, entries_per_class, app_ids_per_class)
	"""

	all_app_ids = ids_data.get_app_ids()
	found_app_ids = set()
	entries_per_app_id = {}
	elements_per_class_per_app_id = {}

	all_classes = ids_data.get_labels()
	found_classes = set()
	entries_per_class = {}
	app_ids_per_class = {}

	for app_id in all_app_ids:
		entries_per_app_id[app_id] = []
		elements_per_class_per_app_id[app_id] = {}

	for a_class in all_classes:
		entries_per_class[a_class] = []
		app_ids_per_class[a_class] = set()

	for entry in log_entries:
		if not entry.intrusion:
			raise NotImplementedError("Entries without labels can currently not be handled")

		app_id = ids_tools.log_entry_to_app_id(entry)
		its_class = entry.intrusion

		found_app_ids.add(app_id)
		found_classes.add(its_class)

		entries_per_app_id[app_id].append(entry)

		if its_class not in elements_per_class_per_app_id[app_id]:
			elements_per_class_per_app_id[app_id][its_class] = 1
		else:
			elements_per_class_per_app_id[app_id][its_class] += 1

		entries_per_class[its_class].append(entry)
		app_ids_per_class[its_class].add(app_id)

	return (found_app_ids, entries_per_app_id, elements_per_class_per_app_id,
		found_classes, entries_per_class, app_ids_per_class)


def _split_log_entries_flow(log_entries, split):
	""" Split the given log entries equally by app_id and each app_id's class.
	Updates the user about progress and success. """

	print("Trying to split the entries according to given split of {}/{}".format(split, 100 - split))

	# { app_id : { class : [entries] } }
	entries_per_app_id_per_class = {}

	# Sort items into buckets
	for log_entry in log_entries:
		app_id = ids_tools.log_entry_to_app_id(log_entry)
		its_class = log_entry.intrusion

		if app_id not in entries_per_app_id_per_class:
			entries_per_app_id_per_class[app_id] = {}
		if its_class not in entries_per_app_id_per_class[app_id]:
			entries_per_app_id_per_class[app_id][its_class] = []

		entries_per_app_id_per_class[app_id][its_class].append(log_entry)

	result_train = []
	result_score = []

	# Split each bucket and add to the result
	for app_id in entries_per_app_id_per_class:
		for a_class in entries_per_app_id_per_class[app_id]:
			items = entries_per_app_id_per_class[app_id][a_class]

			its_split = int((split / 100.0) * len(items))

			result_train += items[:its_split]
			result_score += items[its_split:]

	achieved_split = round((len(result_train) / float(len(log_entries))) * 100, 2)
	print("Done. Achieved a split of {}/{}".format(achieved_split, 100 - achieved_split))
	return result_train, result_score


def _read_file_flow(file_path):
	""" Read the given file as LogEntry objects. Updates the user about the progress. """

	log_entries = []

	if _has_pickle_suffix(file_path):
		print("Using pickle file \"{}\"".format(os.path.join(os.getcwd(), file_path)))
		print("Reading file and verifying contents...")
		log_entries = _get_log_entries_from_pickle(file_path)
	else:
		print("Using log file \"{}\"".format(os.path.join(os.getcwd(), file_path)))
		print("Reading file and converting lines to LogEntry objects...")
		log_entries = _get_log_entries_from_file(file_path)

	print("Done.")
	return log_entries


def _get_log_entries_from_file(file_path):
	lines = Dir.read_lines(file_path)
	# Remove newline at the end of the line and create LogEntry objects
	return [LogEntry.from_log_string(line) for line in lines]


def _get_log_entries_from_pickle(file_path):
	with open(file_path, "r") as pickle_file:
		result = cPickle.load(pickle_file)
		if not isinstance(result[0], LogEntry):
			raise ValueError("Given pickle file does not contain log entries.")
		return result


def _has_pickle_suffix(file_path):
	return file_path.endswith(_PICKLE_SUFFIX)


def _pickle_entries_flow(log_entries, file_path):
	""" Stores the given LogEntry objects in a pickle. Updates the user about the progress. """

	file_path += _PICKLE_SUFFIX

	if os.path.lexists(file_path):
		raise IOError("File {} exists already - delete and try again.".format(file_path))

	print("Saving {} entries to {}...".format(len(log_entries), file_path))

	with open(file_path, "w") as file_handle:
		cPickle.dump(log_entries, file_handle)

	print("Done.")


def _print_table(list_of_lists, headline=None, head_sep=True):
	""" Print the given list of tuple as a table, regarding the first entry the header. """

	if len(list_of_lists) < 2:
		raise ValueError("Input list needs at least one header and one content line!")

	table_column_count = len(list_of_lists[0])
	max_width_per_column = [0] * table_column_count

	for one_list in list_of_lists:
		if len(one_list) != table_column_count:
			raise ValueError("One or more of the given input lines have different numbers of entries!")

		for i in range(0, len(one_list)):
			max_width_per_column[i] = max(max_width_per_column[i], len(str(one_list[i])))

	lines_to_print = []
	col_separator = " | "
	for one_list in list_of_lists:
		justed_strings = []
		for i in range(0, len(one_list)):
			justed_strings.append((str(one_list[i]).ljust(max_width_per_column[i])))
		lines_to_print.append(col_separator.join(justed_strings))

	# Each column plus 3 (" | ")
	table_width = sum(max_width_per_column) + len(col_separator) * (table_column_count - 1)

	# Print #

	print("")
	if headline:
		if not isinstance(headline, str):
			raise ValueError("headline must be string")

		print(headline.center(table_width))

	print(lines_to_print[0])

	if head_sep:
		print("-" * table_width)

	for i in range(1, len(lines_to_print)):
		print(lines_to_print[i])



if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		SUBPARSERS = PARSER.add_subparsers()

		TRAIN_PARSER = SUBPARSERS.add_parser("train", help="Train the classifier")
		TRAIN_PARSER.add_argument("train_file_path", metavar="PATH", help="The training data")
		TRAIN_PARSER.add_argument("--extend-models", "-e", action="store_true", dest="extend_models",
			help="Allow existing models to be extended.")
		TRAIN_PARSER.add_argument("--multiclass", "-m", action="store_true", dest="multi_class")
		TRAIN_PARSER.set_defaults(function=train_call)

		SCORE_PARSER = SUBPARSERS.add_parser("score", help="Score the predictions of the current models")
		SCORE_PARSER.add_argument("test_file_path", metavar="PATH", help="The test data")
		SCORE_PARSER.add_argument("--multiclass", "-m", action="store_true", dest="multi_class")
		SCORE_PARSER.set_defaults(function=score_call)

		TRAINSCORE_PARSER = SUBPARSERS.add_parser("train-and-score", help="Split, train and score")
		TRAINSCORE_PARSER.add_argument("file_path", metavar="PATH", help="The data")
		TRAINSCORE_PARSER.add_argument("--split", "-s", type=int, default=80,
			help="The percentage of data points to be used for training.")
		TRAINSCORE_PARSER.add_argument("--multiclass", "-m", action="store_true", dest="multi_class")
		TRAINSCORE_PARSER.set_defaults(function=train_score_call)

		CONV_PARSER = SUBPARSERS.add_parser("convert", help="Convert log files")
		CONV_PARSER.add_argument("file_path", metavar="PATH")
		MODE_GROUP = CONV_PARSER.add_mutually_exclusive_group(required=True)
		MODE_GROUP.add_argument("--pickle", "-p", action="store_true")
		MODE_GROUP.add_argument("--split", "-s", type=int)
		CONV_PARSER.set_defaults(function=convert_call)

		RESET_PARSER = SUBPARSERS.add_parser("reset", help="Reset the classifier")
		RESET_PARSER.set_defaults(function=reset_call)

		ANAL_PARSER = SUBPARSERS.add_parser("analyse", help="Analyse existing log data")
		ANAL_PARSER.add_argument("file_path", metavar="PATH", help="The file to analyse")
		ANAL_PARSER.set_defaults(function=anal_call)

		ARGS = PARSER.parse_args()

		START_TIME = time.time()
		# Actual functionality
		ARGS.function(ARGS)
		TIME_EXPIRED = time.time() - START_TIME
		print("Finished in {}".format(ids_tools.format_time_passed(TIME_EXPIRED)))
		exit()
	except KeyboardInterrupt:
		pass
