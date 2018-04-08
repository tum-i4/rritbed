#!/usr/bin/env python
""" a """

# pylint: disable-msg=C0103; (Invalid name)

import argparse

from log_entry import LogEntry
from ids.ids_entry import IdsEntry
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

	# ids_entries: { app_id, vector, my_class }
	ids_entries = read_convert(file_path)

	ids_entries_per_app = _empty_app_id_dict()
	for ids_entry in ids_entries:
		ids_entries_per_app[ids_entry.app_id].append(ids_entry)

	for app_id, ids_entries in ids_entries_per_app.items():
		b(app_id, ids_entries)


def b(app_id, ids_entries):
	""" Pre-processes the given entries. """

	if not isinstance(ids_entries[0], IdsEntry):
		raise TypeError("Given list does not contain IdsEntry objects.")

	print("NO PREPROCESSING")
	c(app_id, ids_entries)


def c(app_id, ids_entries):
	""" Fit, score, print. """

	if not isinstance(ids_entries[0], IdsEntry):
		raise TypeError("Given list does not contain IdsEntry objects.")

	printer = util.prtr.Printer(name=app_id)
	squelcher = util.prtr.Printer(squelch=True)
	converter = IdsConverter()

	printer.prt("Splitting... ")
	train_entries, test_entries = ids_tools.ids_entries_to_train_test(ids_entries)
	train_dict = converter.ids_entries_to_train_dict(train_entries, squelcher)
	test_dict = converter.ids_entries_to_train_dict(test_entries, squelcher)

	if len(train_dict) > 1 or len(test_dict) > 1:
		raise NotImplementedError("Method is built for converted entries of one app only!")

	raise NotImplementedError()


### Helpers ###


def read_convert(file_path):
	""" Read log entries from the given file and convert the result. """

	converter = IdsConverter()
	ids_entries = []

	for line in Dir.yield_lines(file_path, 5000000):
		log_entry = LogEntry.from_log_string(line)
		ids_entry = converter.log_entry_to_ids_entry(log_entry, binary=True)
		ids_entries.append(ids_entry)

	return ids_entries


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
