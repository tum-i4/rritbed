#!/usr/bin/env python

import argparse

from log_entry import LogEntry
import util.fmtr
import util.prtr
import ids.ids_tools as ids_tools
from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter


def a(file_path):

	printer = util.prtr.Printer(name="a")
	squelcher = util.prtr.Printer(squelch=True)
	converter = IdsConverter()

	printer.prt("Reading file and converting...")

	# converted_entries: [(app_id, vector, class)]
	converted_entries = read_convert(file_path)

	scores_acc = _empty_app_id_dict()
	scores_prec = _empty_app_id_dict()
	scores_rec = _empty_app_id_dict()

	printer.prt("Splitting... ")
	train_entries, test_entries = ids_tools.converted_entries_to_train_test(converted_entries)
	train_dict = converter.prepared_tuples_to_train_dict(train_entries, squelcher)
	test_dict = converter.prepared_tuples_to_train_dict(test_entries, squelcher)

	for app_id in converter.app_ids:
		pass

	raise NotImplementedError()


def read_convert(file_path):
	""" Read log entries from the given file and convert the result. """

	converter = IdsConverter()
	converted_entries = []

	for line in Dir.yield_lines(file_path, 5000000):
		log_entry = LogEntry.from_log_string(line)
		converted_entry = converter.log_entry_to_prepared_tuple(log_entry, binary=True)
		converted_entries.append(converted_entry)

	return converted_entries


def _empty_app_id_dict():
	""" Initialises an empty dict with { app_id: [] }. """

	result = {}
	for app_id in IdsConverter().app_ids:
		result[app_id] = []

	return result


if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		PARSER.add_argument("file_path", metavar="PATH/FILE", help="Log file")

		ARGS = PARSER.parse_args()

		a(ARGS.file_path)

		exit()
	except KeyboardInterrupt:
		pass
