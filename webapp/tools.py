#!/usr/bin/env python
""" Tools for command-line interaction with the server """

import argparse
import os

from log_entry import LogEntry
from ids.intrusion_classifier import IntrusionClassifier
from ids.dir_utils import Dir
import ids.ids_tools as ids_tools
import ids.ids_data as ids_data


def train_call(args):
	""" Unpack the args and call _train. """
	_train(args.train_file_path, args.extend_models)
	exit()


def _train(file_path, extend_models=False):
	""" Train the classifier with the given file. Optionally allow extension of models. """

	history_file = "intrusion_classifier_history"
	saved_so_far = []

	if os.path.lexists(history_file):
		saved_so_far = Dir.read_lines(history_file)

	if file_path in saved_so_far:
		print("This file has already been used for training."
			+ " If you think this is a mistake, rename it and run again.")
		exit()

	print("Using file \"{}\"".format(os.path.join(os.getcwd(), file_path)))
	print("Reading file and converting lines to LogEntry objects...")
	log_entries = _get_log_entries_from_file(file_path)
	print("Done.")

	clas = IntrusionClassifier.get_singleton()
	try:
		clas.train(log_entries, extend_models=extend_models)
	except ValueError as val_err:
		print(val_err.message)
		return


def anal_call(args):
	""" Call _analyse. """
	_analyse(args.file_path)
	exit()


def _analyse(file_path):
	""" Analyse the given log file. """
	print("Analysing...")

	# Reading file #

	log_entries = _get_log_entries_from_file(file_path)

	# Analysis #

	found_app_ids = set()
	all_app_ids = ids_data.get_app_ids()
	entries_per_app_id = {}
	elements_per_class_per_app_id = {}

	found_classes = set()
	all_classes = ids_data.get_labels()
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

	# Output #

	get_pl = lambda s, obj: s if len(obj) > 1 else ""

	print("")
	print("Found {} entries with {}/{} app id{} and {}/{} class{}".format(
		len(log_entries),
		len(found_app_ids), len(all_app_ids), get_pl("s", found_app_ids),
		len(found_classes), len(all_classes), get_pl("es", found_classes))
	)

	# App ID table

	max_len = lambda seq: max([len(x) for x in seq])
	max_str_len = lambda seq: max([len(str(len(x))) for x in seq])
	header_class = "Class"
	header_elements = "Elements"

	per_app_id_table = []
	per_app_id_table.append(["App ID", "Elements"])
	for app_id in all_app_ids:
		per_app_id_table.append([app_id, str(len(entries_per_app_id[app_id]))])

	_print_table(per_app_id_table)

	# Classes per app ID tables

	for app_id in all_app_ids:
		classes = elements_per_class_per_app_id[app_id].keys()
		counts = elements_per_class_per_app_id[app_id].values()
		classes_per_app_id_table = [classes]
		classes_per_app_id_table.append(counts)
		_print_table(classes_per_app_id_table, headline=app_id, head_sep=False)

	# Class table

	class_col = max(len(header_class), max_len(all_classes))
	cls_el_count_col = max(len(header_elements), max_str_len(entries_per_class.values()))

	# Header
	header_line_classes = "{} | {} | {}".format(
		header_class.ljust(class_col),
		header_elements.ljust(cls_el_count_col),
		"App Ids")
	print("")
	print("Per class".center(len(header_line_classes)))
	print(header_line_classes)
	print("-" * len(header_line_classes))
	for a_class in all_classes:
		print("{} | {} | {}".format(
			a_class.ljust(class_col),
			str(len(entries_per_class[a_class])).ljust(cls_el_count_col),
			len(app_ids_per_class[a_class]))
		)

	# per app id: entries per class

	# TODO: score??
	# harmonious? all labelled / some / none?
	# for each app id: are there roughly the same number of entries per class?
	return


def _get_log_entries_from_file(file_path):
	lines = Dir.read_lines(file_path)
	# Remove newline at the end of the line and create LogEntry objects
	return [LogEntry.from_log_string(line[:-1]) for line in lines]


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
		TRAIN_PARSER.add_argument("train_file_path", metavar="PATH",
			help="File to train the classifier with.")
		TRAIN_PARSER.add_argument("--extend-models", "-e", action="store_true", dest="extend_models",
			help="Allow existing models to be extended.")
		TRAIN_PARSER.set_defaults(function=train_call)

		ANAL_PARSER = SUBPARSERS.add_parser("analyse", help="Analyse existing log data")
		ANAL_PARSER.add_argument("file_path", metavar="PATH", help="The file to analyse")
		ANAL_PARSER.set_defaults(function=anal_call)

		ARGS = PARSER.parse_args()

		ARGS.function(ARGS)
	except KeyboardInterrupt:
		pass
