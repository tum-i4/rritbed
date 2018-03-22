#!/usr/bin/env python
""" Tools for command-line interaction with the server """

import argparse
import cPickle
import md5
import os
import statistics as stat
import sys
import time

from log_entry import LogEntry
from state_dao import StateDao
from ids.intrusion_classifier import IntrusionClassifier
from ids.dir_utils import Dir, ModelDir
import ids.ids_tools as ids_tools
import ids.ids_data as ids_data


_PICKLE_SUFFIX = ".pickle"
_HISTORY_FILE = "intrusion_classifier_history"


def train_call(args):
	""" Unpack the args and call _train.
	Expects 'train_file_path' and 'extend_models'. """
	_train(args.train_file_path, args.extend_models)


def _train(file_path, extend_models):
	""" Train the classifier with the given file. Optionally allow extension of models. """

	print("Using file \"{}\"".format(os.path.join(os.getcwd(), file_path)))

	saved_so_far = []

	if os.path.lexists(_HISTORY_FILE):
		saved_so_far = Dir.read_lines(_HISTORY_FILE)

	if file_path in saved_so_far:
		print("This file has already been used for training."
			+ " If you think this is a mistake, rename it and run again.")
		return

	log_entry_generator = _yield_log_entries_from_file(file_path)
	_train_entries(log_entry_generator, extend_models)

	with open(_HISTORY_FILE, 'a') as hist_file:
		hist_file.write(file_path + "\n")


def _train_entries(log_entry_generator, extend_models, squelch_output=False):
	"""
	Train with the given LogEntry objects.
	returns: Boolean flag indicating success
	"""

	clas = IntrusionClassifier.get_singleton()

	try:
		clas.train(log_entry_generator, extend_models=extend_models, squelch_output=squelch_output)
		return True
	except ValueError as val_err:
		print(val_err.message)
		return False


def score_call(args):
	""" Unpack the args and call _score.
	Expects 'test_file_path' and 'iterations'. """
	_score(args.test_file_path, args.iterations)


def _score(file_path, iterations):
	""" Score the prediction of the classifier with the given test file. """

	log_entries = _read_file_flow(file_path)
	if iterations == 1:
		_score_entries(log_entries)
	elif iterations > 1:
		_score_entries_in_iterations(log_entries, iterations)
	else:
		print("Iterations must be a positive value! Got: {}".format(iterations))


def _score_entries_in_iterations(log_entries, iterations):
	""" Score the given LogEntry objects in multiple iterations and print the output. """

	scores = {}
	for app_id in ids_data.get_app_ids():
		scores[app_id] = []

	printer = ids_tools.Printer()

	printer.prt("Scoring in {} iterations: ".format(iterations), newline=False)

	for i in range(iterations, 0, -1):
		# Score
		printer.prt("{}...".format(i), newline=False)
		scoring_result = _score_entries(log_entries,
			do_return=True, squelch_output=True)
		if not scoring_result:
			printer.prt("")
			printer.prt("Scoring failed!")
			# Don't continue; reset needs to happen in order to allow for the next iteration

		for app_id in scoring_result:
			scores[app_id].append(scoring_result[app_id])

	printer.prt("Done!")

	_print_scores(scores, printer)


def _score_entries(log_entries, do_return=False, squelch_output=False):
	"""
	Score the given LogEntry objects.
	returns: Boolean flag indicating success
	"""

	clas = IntrusionClassifier.get_singleton()

	try:
		result = clas.score(log_entries, do_return=do_return, squelch_output=squelch_output)
		return result if do_return else True
	except ValueError as val_err:
		print(val_err.message)
		return None if do_return else False


def train_score_call(args):
	""" Unpack the args and call _train_and_score.
	Expects 'file_path', 'split' and 'iterations' """
	_train_and_score(args.file_path, args.split, args.iterations)


def _train_and_score(file_path, split, iterations):
	""" Split the given file and use the first part for training, the second for scoring. """

	if split <= 0 or split >= 100:
		raise ValueError("Invalid split \"{}\" given.".format(split))

	log_entries = _read_file_flow(file_path)

	if len(log_entries) < 10000:
		raise IOError("Insufficient number of entries found in the file. Need >= 10,000.")

	scores = {}
	for app_id in ids_data.get_app_ids():
		scores[app_id] = []

	printer = ids_tools.Printer()

	for i in range(1, iterations + 1):
		printer.prt("Iteration {} of {}.".format(i, iterations))

		# Split
		printer.prt("Splitting... ", newline=False)
		training_entries, scoring_entries = _split_log_entries_flow(
			log_entries, split, squelch_output=True)

		preconditions_msg = "Please make sure that all preconditions are met and rerun."

		# Train
		printer.prt("Training... ", newline=False)
		training_succeeded = _train_entries(training_entries,
			extend_models=False, squelch_output=True)
		if not training_succeeded:
			printer.prt("")
			printer.prt("Training failed. " + preconditions_msg)
			continue

		# Score
		printer.prt("Scoring... ", newline=False)
		scoring_result = _score_entries(scoring_entries,
			do_return=True, squelch_output=True)
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


def _print_scores(scores, printer):
	""" Print the given scores in a table. """

	if not scores or not scores.itervalues().next():
		printer.prt("No results!")
		return

	result_table = []

	if len(scores.itervalues().next()) == 1:
		result_table.append(["Classifier", "Score"])
		for app_id in scores:
			result_table.append([app_id, ids_tools.format_percentage(scores[app_id][0], True)])
	else:
		result_table.append(["Classifier", "Avg. score", "Variance", "", "All scores"])
		for app_id in scores:
			row = scores[app_id]
			result_table.append([
				app_id,
				ids_tools.format_percentage(ids_tools.avg(row)),
				round(stat.variance([x * 100 for x in row]), 2),
				"",
				", ".join([ids_tools.format_percentage(x, pad_spaces=True) for x in row])
			])

	_print_table(result_table, headline="Results")


def split_call(args):
	""" Unpack the args and call the respective _split_*.
	Expects 'file_path', 'max-entries-per-file'
	and 'train_split' / 'split_per_app_id' / 'split_in_chunks'. """

	if args.max_entries_per_file <= 0:
		raise ValueError("Max entries need to be > 0. Received: {}".format(args.max_entries_per_file))

	log_entry_generator = _yield_log_entries_from_file(args.file_path)

	if args.train_split:
		_split_in_train_and_score(log_entry_generator, args.file_path, args.train_split)
	elif args.per_app_id:
		_split_per_app_id(log_entry_generator, args.file_path, args.max_entries_per_file)
	elif args.in_chunks:
		_split_in_chunks(log_entry_generator, args.file_path, args.max_entries_per_file)
	else:
		raise NotImplementedError("Arg configuration not implemented")


def _split_in_train_and_score(log_entry_generator, file_path, split):
	""" Split the given entries into a training and a scoring file based on the given split. """

	print("WARNING: --max-entries-per-file is not implemented")

	if split <= 0 or split >= 100:
		raise ValueError("Invalid split \"{}\" given.".format(split))

	training_entries, scoring_entries = _split_log_entries_flow(log_entry_generator, split)

	training_file_path = file_path + "_train"
	scoring_file_path = file_path + "_score"

	try:
		_save_entries_flow(training_entries, training_file_path)
		_save_entries_flow(scoring_entries, scoring_file_path)
	except IOError as io_err:
		print(io_err.message)
		return

	print("Split was finished successfully!")
	return


def _split_per_app_id(log_entry_generator, file_path, max_per_file=1000000):
	""" Split the given entries into separate files for each app id. """

	print("Splitting the given file per app id...")

	file_path_no_ext, ext = os.path.splitext(file_path)

	all_app_ids = ids_data.get_app_ids()
	file_handles = dict()
	entry_counts = dict.fromkeys(all_app_ids, 0)
	file_indices = dict.fromkeys(all_app_ids, 0)

	try:
		for app_id in all_app_ids:
			new_path = file_path_no_ext + "_" + app_id + ext
			file_handles[app_id] = _init_file_handle(new_path)

		for log_entry in log_entry_generator:
			app_id = ids_tools.log_entry_to_app_id(log_entry)

			if entry_counts[app_id] == max_per_file:
				print("Entry count for {} has reached allowed maximum of {}. Creating a new file..."
					.format(app_id, max_per_file))

				entry_counts[app_id] = 0
				file_indices[app_id] += 1

				new_index = file_indices[app_id]
				new_path = file_path_no_ext + "_" + app_id + "_" + new_index + ext

				file_handles[app_id].close()
				file_handles[app_id] = _init_file_handle(new_path)


			file_handles[app_id].write(log_entry.get_log_string() + "\n")
			entry_counts[app_id] += 1
	except IOError as io_err:
		print(io_err.message)
		return
	finally:
		for file_handle in file_handles.values():
			file_handle.close()

		print("Closed all file handles.")

	print("Done.")


def _split_in_chunks(log_entry_generator, file_path, max_entries_per_file):
	""" Split the given entries into chunks of size <max_entries_per_file>. """

	file_path_no_ext, ext = os.path.splitext(file_path)
	path_creator = lambda idx: file_path_no_ext + "_" + str(idx) + ext
	writer_msg = lambda idx: "Writing file {}...".format(path_creator(idx))

	current_count = 0
	current_index = 1
	current_file = _init_file_handle(path_creator(current_index))

	print(writer_msg(current_index))

	for log_entry in log_entry_generator:
		if current_count == max_entries_per_file:
			current_count = 0
			current_index += 1
			current_file.close()

			print(writer_msg(current_index))
			current_file = _init_file_handle(path_creator(current_index))

		current_file.write(log_entry.get_log_string() + "\n")
		current_count += 1

	current_file.close()
	print("Done.")


def _init_file_handle(path):
	""" Check if the given file exists and open it if it does. """

	if os.path.lexists(path):
		raise IOError("File {} exists - please delete and try again.".format(path))

	return open(path, "w")


# pylint: disable-msg=W0613; (Unused argument)
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


def anal_call(args):
	""" Unpack the args and call _analyse.
	Expects 'file_path'. """
	_analyse(args.file_path)


def _analyse(file_path):
	""" Analyse the given log file. """
	print("Analysing...")

	# Get file access #

	log_entry_generator = _yield_log_entries_from_file(file_path)

	# Analysis #

	all_app_ids = ids_data.get_app_ids()
	all_classes = ids_data.get_labels()

	(
		total_entries, found_app_ids, entry_count_per_app_id, elements_per_class_per_app_id,
		found_classes, entry_count_per_class, app_ids_per_class, duplicate_elements_per_app_id
	) = _analyse_entries(log_entry_generator)

	# Output #

	get_pl = lambda s, obj: s if len(obj) > 1 else ""

	print("")
	print("Found {} entries with {}/{} app id{} and {}/{} class{}".format(
		total_entries,
		len(found_app_ids), len(all_app_ids), get_pl("s", found_app_ids),
		len(found_classes), len(all_classes), get_pl("es", found_classes))
	)

	# App ID table
	per_app_id = []
	per_app_id.append(["App ID", "Elements"] + all_classes)
	for app_id in all_app_ids:
		line = [app_id, str(entry_count_per_app_id[app_id])]

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
		per_class.append([a_class, entry_count_per_class[a_class], len(app_ids_per_class[a_class])])

	_print_table(per_class, headline="Elements per class")

	# Duplicate table
	duplicates = []
	duplicates.append(["App ID", "All", "Unique", "Duplicates", "Duplicate %"])
	for app_id in all_app_ids:
		result = duplicate_elements_per_app_id[app_id]
		unique_count = result["uniq"]
		duplicate_count = result["dupe"]
		all_count = unique_count + duplicate_count
		duplicate_percent_str = ids_tools.format_percentage(0)
		if all_count > 0:
			duplicate_percent_str = ids_tools.format_percentage(
				float(duplicate_count) / all_count)

		duplicates.append([app_id, all_count, unique_count, duplicate_count, duplicate_percent_str])

	# Check content (skip header) for found duplicates
	if not any([l[3] > 0 for l in duplicates[1:]]):
		print("\nDuplicate analysis: No duplicates found!")
	else:
		_print_table(duplicates, headline="Duplicates per app ID")

	# TODO: score??
	# harmonious? all labelled / some / none?
	# for each app id: are there roughly the same number of entries per class?
	return


def _analyse_entries(log_entry_generator):
	"""
	Analyse the LogEntry objects from the given generator.
	returns: A tuple containing (found_app_ids, entry_count_per_app_id, elements_per_class_per_app_id,
	found_classes, entry_count_per_class, app_ids_per_class, duplicate_elements_per_app_id)
	"""

	total_entries = 0

	all_app_ids = ids_data.get_app_ids()
	found_app_ids = set()
	entry_count_per_app_id = {}
	elements_per_class_per_app_id = {}

	all_classes = ids_data.get_labels()
	found_classes = set()
	entry_count_per_class = {}
	app_ids_per_class = {}

	duplicate_elements_per_app_id = {}
	last_hash_per_app_id = {}

	for app_id in all_app_ids:
		entry_count_per_app_id[app_id] = 0
		elements_per_class_per_app_id[app_id] = {}

		# Unique, Duplicates
		duplicate_elements_per_app_id[app_id] = dict(uniq=0, dupe=0)
		last_hash_per_app_id[app_id] = None

	for a_class in all_classes:
		entry_count_per_class[a_class] = 0
		app_ids_per_class[a_class] = set()

	for entry in log_entry_generator:
		if not entry.intrusion:
			raise NotImplementedError("Entries without labels can currently not be handled")

		total_entries += 1

		app_id = ids_tools.log_entry_to_app_id(entry)
		its_class = entry.intrusion

		found_app_ids.add(app_id)
		found_classes.add(its_class)

		entry_count_per_app_id[app_id] += 1

		if its_class not in elements_per_class_per_app_id[app_id]:
			elements_per_class_per_app_id[app_id][its_class] = 1
		else:
			elements_per_class_per_app_id[app_id][its_class] += 1

		entry_count_per_class[its_class] += 1
		app_ids_per_class[its_class].add(app_id)

		entry_hash = _get_content_hash(entry)
		if entry_hash == last_hash_per_app_id[app_id]:
			duplicate_elements_per_app_id[app_id]["dupe"] += 1
		else:
			duplicate_elements_per_app_id[app_id]["uniq"] += 1
		last_hash_per_app_id[app_id] = entry_hash


	return (total_entries, found_app_ids, entry_count_per_app_id, elements_per_class_per_app_id,
		found_classes, entry_count_per_class, app_ids_per_class, duplicate_elements_per_app_id)


def _get_content_hash(log_entry):
	""" Hash only the content of the given log entry.
	Omits app_id, time_unix and log id. Includes label. """

	entry_string = log_entry.data[LogEntry.VIN_FIELD]
	entry_string += log_entry.data[LogEntry.LEVEL_FIELD]
	entry_string += log_entry.data[LogEntry.GPS_POSITION_FIELD]
	entry_string += log_entry.data[LogEntry.LOG_MESSAGE_FIELD]
	entry_string += log_entry.intrusion

	return md5.new(entry_string).hexdigest()


def _split_log_entries_flow(log_entry_iterator, split, squelch_output=False):
	""" Split the given log entries equally by app_id and each app_id's class.
	Updates the user about progress and success. """

	printer = ids_tools.Printer(squelch=squelch_output)

	printer.prt("Trying to split the entries according to given split of {}/{}"
		.format(split, 100 - split))

	# { app_id : { class : [entries] } }
	entries_per_app_id_per_class = {}

	entry_count = 0

	# Sort items into buckets
	for log_entry in log_entry_iterator:
		entry_count += 1

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

	achieved_split = round((len(result_train) / float(entry_count)) * 100, 2)
	printer.prt("Done. Achieved a split of {}/{}".format(achieved_split, 100 - achieved_split))
	return result_train, result_score


def _read_file_flow(file_path, squelch_output=False):
	""" Read the given file as LogEntry objects. Updates the user about the progress. """

	printer = ids_tools.Printer(squelch=squelch_output)

	log_entries = []

	if _has_pickle_suffix(file_path):
		printer.prt("Using pickle file \"{}\"".format(os.path.join(os.getcwd(), file_path)))
		printer.prt("Reading file and verifying contents...")
		log_entries = _get_log_entries_from_pickle(file_path)
	else:
		printer.prt("Using log file \"{}\"".format(os.path.join(os.getcwd(), file_path)))
		printer.prt("Reading file and converting lines to LogEntry objects...")
		log_entries = _get_log_entries_from_file(file_path)

	printer.prt("Done.")
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


def _yield_log_entries_from_file(file_path):
	for line in Dir.yield_lines(file_path):
		yield LogEntry.from_log_string(line)


def _has_pickle_suffix(file_path):
	return file_path.endswith(_PICKLE_SUFFIX)


def _save_entries_flow(log_entry_iterator, file_path):
	""" Stores the given LogEntry objects in a log file. Updates the user about the progress. """

	if os.path.lexists(file_path):
		raise IOError("File {} exists already - delete and try again.".format(file_path))

	print("Saving entries to {}...".format(file_path))

	with open(file_path, "w") as file_handle:
		for log_entry in log_entry_iterator:
			file_handle.write(log_entry.get_log_string() + "\n")

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
		TRAIN_PARSER.add_argument("--extend-models", "-e", action="store_true",
			help="Allow existing models to be extended.")
		TRAIN_PARSER.set_defaults(function=train_call)

		SCORE_PARSER = SUBPARSERS.add_parser("score", help="Score the predictions of the current models")
		SCORE_PARSER.add_argument("test_file_path", metavar="PATH", help="The test data")
		SCORE_PARSER.add_argument("--iterations", "-i", type=int, default=1)
		SCORE_PARSER.set_defaults(function=score_call)

		TRAINSCORE_PARSER = SUBPARSERS.add_parser("train-and-score", help="Split, train, score, reset")
		TRAINSCORE_PARSER.add_argument("file_path", metavar="PATH", help="The data")
		TRAINSCORE_PARSER.add_argument("--split", "-s", type=int, default=80,
			help="The percentage of data points to be used for training.")
		TRAINSCORE_PARSER.add_argument("--iterations", "-i", type=int, default=1)
		TRAINSCORE_PARSER.set_defaults(function=train_score_call)

		SPLIT_PARSER = SUBPARSERS.add_parser("split", help="Split a log file")
		SPLIT_PARSER.add_argument("file_path", metavar="PATH")
		MODE_GROUP = SPLIT_PARSER.add_mutually_exclusive_group(required=True)
		MODE_GROUP.add_argument("--train-and-score", "-t", type=int, dest="train_split",
			help="Split into train and score file based on the given percentage.")
		MODE_GROUP.add_argument("--per-app-id", "-i", action="store_true",
			help="Split into sub-files containing entires separated by app id.")
		MODE_GROUP.add_argument("--in-chunks", "-c", action="store_true",
			help="Split into chunks. Specify the number of chunks with --max-entries-per-file/-m.")
		DEFAULT_MEPF = 1000000
		SPLIT_PARSER.add_argument("--max-entries-per-file", "-m", type=int, default=DEFAULT_MEPF,
			help="Limit the number of entries saved per file (default {})".format(DEFAULT_MEPF))
		SPLIT_PARSER.set_defaults(function=split_call)

		RESET_PARSER = SUBPARSERS.add_parser("reset", help="Reset the classifier")
		RESET_PARSER.add_argument("--classifier", "-c", action="store_true")
		RESET_PARSER.add_argument("--server-log", "-l", action="store_true")
		RESET_PARSER.add_argument("--all", "-a", action="store_true")
		RESET_PARSER.set_defaults(function=reset_call)

		ANAL_PARSER = SUBPARSERS.add_parser("analyse", help="Analyse existing log data")
		ANAL_PARSER.add_argument("file_path", metavar="PATH", help="The file to analyse")
		ANAL_PARSER.set_defaults(function=anal_call)

		ARGS = PARSER.parse_args()

		START_TIME = time.time()
		# Actual functionality
		ARGS.function(ARGS)
		TIME_EXPIRED = time.time() - START_TIME
		print("")
		print("Finished task '{}' in {}".format(
			sys.argv[1],
			ids_tools.format_time_passed(TIME_EXPIRED)))
		exit()
	except KeyboardInterrupt:
		pass
