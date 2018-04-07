#!/usr/bin/env python

# pylint: disable-msg=C0103; (Invalid name)

import argparse

from log_entry import LogEntry
import util.fmtr
import util.prtr
import ids.ids_tools as ids_tools
from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter


### Workers ###


def a(file_path):
	""" Read entries, convert them, split them per app_id and call b() for each app. """

	printer = util.prtr.Printer(name="a")
	printer.prt("Reading file and converting...")

	# converted_entries: [(app_id, vector, class)]
	converted_entries = read_convert(file_path)

	entries_per_app = _empty_app_id_tuple_dict()
	for app_id, vector, its_class in converted_entries:
		entries_per_app[app_id][0].append(vector)
		entries_per_app[app_id][1].append(its_class)

	for app_id, (X, y) in entries_per_app.items():
		b(X, y)


def b(app_id, converted_entries):

	printer = util.prtr.Printer(name=app_id)
	squelcher = util.prtr.Printer(squelch=True)
	converter = IdsConverter()

	printer.prt("Splitting... ")
	train_entries, test_entries = ids_tools.converted_entries_to_train_test(converted_entries)
	train_dict = converter.prepared_tuples_to_train_dict(train_entries, squelcher)
	test_dict = converter.prepared_tuples_to_train_dict(test_entries, squelcher)

	raise NotImplementedError()


### Helpers ###


def read_convert(file_path):
	""" Read log entries from the given file and convert the result. """

	converter = IdsConverter()
	converted_entries = []

	for line in Dir.yield_lines(file_path, 5000000):
		log_entry = LogEntry.from_log_string(line)
		converted_entry = converter.log_entry_to_prepared_tuple(log_entry, binary=True)
		converted_entries.append(converted_entry)

	return converted_entries


def _empty_app_id_tuple_dict():
	""" Initialises an empty dict with { app_id: ([], []) }. """

	result = {}
	for app_id in IdsConverter().app_ids:
		result[app_id] = ([], [])

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
