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
		print(io_err)
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
		print(io_err)
		return
	finally:
		for file_handle in file_handles.values():
			file_handle.close()

		print("Closed all file handles.")

	print("Done.")


def _split_in_chunks(log_entry_generator, file_path, max_entries_per_file):
	""" Split the given entries into chunks of size <max_entries_per_file>. """

	if max_entries_per_file is None:
		raise ValueError("Please specify the number of entries per file!")

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


def sample_call(args):
	""" Unpack the args and call _sample.
	Expects 'file_path' and 'number_of_elements', optionally 'limit_to'. """

	if not args.file_path or not args.number_of_elements:
		raise RuntimeError("Missing arg!")

	_sample(args.file_path, args.number_of_elements, args.limit_to)


def _sample(file_path, number_of_elements, limit_to):
	""" Sample <number_of_elements> from the given file. """

	print("Sampling...")

	target_file_path = "%s_%s-sample" % (file_path, number_of_elements)

	if not os.path.lexists(file_path) or os.path.lexists(target_file_path):
		raise IOError("Input file doesn't OR output file does exist")

	line_generator = Dir.yield_lines(file_path)

	log_lines = None
	if limit_to is None:
		log_lines = ids_tools.reservoir_sample(line_generator, number_of_elements)
	else:
		log_lines = ids_tools.reservoir_sample_limit(line_generator, number_of_elements, limit_to)

	Dir.write_lines(target_file_path, log_lines)

	print("Done. Wrote to file:\n%s" % target_file_path)


def analyse_call(args):
	""" Unpack the args and call log_file_analysis.analyse.
	Expects 'file_path' and 'to_file'. """
	log_file_analysis.analyse(args.file_path, args.to_file, util.prtr.Printer())


def _split_log_entries_flow(log_entry_iterator, split, squelch_output=False):
	""" Split the given log entries equally by app_id and each app_id's class.
	Updates the user about progress and success. """

	printer = util.prtr.Printer(squelch=squelch_output)

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


def _yield_log_entries_from_file(file_path):
	for line in Dir.yield_lines(file_path):
		yield LogEntry.from_log_string(line)


def _save_entries_flow(log_entry_iterator, file_path):
	""" Stores the given LogEntry objects in a log file. Updates the user about the progress. """

	if os.path.lexists(file_path):
		raise IOError("File {} exists already - delete and try again.".format(file_path))

	print("Saving entries to {}...".format(file_path))

	with open(file_path, "w") as file_handle:
		for log_entry in log_entry_iterator:
			file_handle.write(log_entry.get_log_string() + "\n")

	print("Done.")



if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		SUBPARSERS = PARSER.add_subparsers()

		SPLIT_PARSER = SUBPARSERS.add_parser("split", help="Split a log file")
		SPLIT_PARSER.add_argument("file_path", metavar="PATH")
		MODE_GROUP = SPLIT_PARSER.add_mutually_exclusive_group(required=True)
		MODE_GROUP.add_argument("--train-and-score", "-t", type=int, dest="train_split",
			help="Split into train and score file based on the given percentage.")
		MODE_GROUP.add_argument("--per-app-id", "-i", action="store_true",
			help="Split into sub-files containing entries separated by app id.")
		MODE_GROUP.add_argument("--in-chunks", "-c", action="store_true",
			help="Split into chunks. Specify the number of entries per file with --max-entries-per-file/-m.")
		SPLIT_PARSER.add_argument("--max-entries-per-file", "-m", type=int,
			help="Limit the number of entries saved per file.Caution: Significantly slower!")
		SPLIT_PARSER.set_defaults(function=split_call)

		SAMPLE_PARSER = SUBPARSERS.add_parser("sample", help="Sample from a log file")
		SAMPLE_PARSER.add_argument("file_path", metavar="PATH")
		SAMPLE_PARSER.add_argument("number_of_elements", type=int,
			help="Sample size. Needs to be smaller than the available lines in the given file.")
		SAMPLE_PARSER.add_argument("--limit-to", "-l", nargs="+", choices=ids_data.get_app_ids(),
			help="Only sample entries of the given data type(s).")
		SAMPLE_PARSER.set_defaults(function=sample_call)

		ANALYSE_PARSER = SUBPARSERS.add_parser("analyse", help="Analyse existing log data")
		ANALYSE_PARSER.add_argument("file_path", metavar="PATH", help="The file to analyse")
		ANALYSE_PARSER.add_argument("--to-file", "-f", action="store_true",
			help="Save the analysis to file.")
		ANALYSE_PARSER.set_defaults(function=analyse_call)

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
