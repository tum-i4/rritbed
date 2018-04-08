#!/usr/bin/env python
""" a """

# pylint: disable-msg=C0103; (Invalid name)

import argparse

import sklearn
import sklearn.metrics as sk_metr

from log_entry import LogEntry
from ids.ids_entry import IdsEntry
import util.fmtr
from util.prtr import TimePrinter
import ids.ids_tools as ids_tools
from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter


### Workers ###


def a(file_path):
	""" Read entries, convert them, split them per app_id and call b() for each app. """

	printer = TimePrinter(name="a")
	printer.prt("Reading file and converting...")

	# ids_entries: { app_id, vector, my_class }
	ids_entries = read_convert(file_path)

	ids_entries_per_app = empty_app_id_dict()
	for ids_entry in ids_entries:
		ids_entries_per_app[ids_entry.app_id].append(ids_entry)

	for app_id, ids_entries in ids_entries_per_app.items():
		b(app_id, ids_entries)


def b(app_id, ids_entries):
	""" Pre-processes the given entries. """

	if not isinstance(ids_entries[0], IdsEntry):
		raise TypeError("Given list does not contain IdsEntry objects.")

	printer = TimePrinter(name="b")

	if not ids_entries:
		printer.prt("No input data for {}".format(app_id))
		return

	# import sklearn.pipeline as sk_pipe
	# import sklearn.preprocessing as sk_pre

	printer.prt("NO PREPROCESSING")
	c(app_id, ids_entries)


def c(app_id, ids_entries):
	""" Fit, predict. """

	if not isinstance(ids_entries[0], IdsEntry):
		raise TypeError("Given list does not contain IdsEntry objects.")

	printer = TimePrinter(name=app_id)

	printer.prt("Splitting... ")
	try:
		train_entries, test_entries = ids_tools.ids_entries_to_train_test(ids_entries)
	except ValueError as val_err:
		print(val_err.message)
		exit()

	X_train, _ = unravel_ids_entries(train_entries, app_id)
	X_test, y_true = unravel_ids_entries(test_entries, app_id)

	printer.prt("Fitting... ")

	clf = sklearn.svm.OneClassSVM(random_state=0)
	clf.fit(X_train)

	printer.prt("Predicting... ")

	y_pred = clf.predict(X_test)

	d(app_id, y_true, y_pred)


def d(app_id, y_true, y_pred):
	""" Score, print. """

	tn, fp, fn, tp = sk_metr.confusion_matrix(y_true, y_pred).ravel()

	for row in [
		("\t", "Actual (+)", "Actual (-)"),
		("Pred (+)", tp, fp),
		("Pred (-)", fn, tn)
	]:
		print("%s\t%s\t\t%s" % row)

	print("WTF")


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


def empty_app_id_dict():
	""" Initialise an empty dict with { app_id: [] }. """

	result = {}
	for app_id in IdsConverter().app_ids:
		result[app_id] = []

	return result


def unravel_ids_entries(ids_entries, expected_app_id):
	""" Convert the given IdsEntry objects to X and y. """

	X = []
	y = []

	for entry in ids_entries:
		if entry.app_id != expected_app_id:
			raise ValueError("Given IdsEntry has an incorrect app_id!")

		X.append(entry.vector)
		y.append(entry.vclass)

	return (X, y)


if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		PARSER.add_argument("file_path", metavar="PATH/FILE", help="Log file")

		ARGS = PARSER.parse_args()

		a(ARGS.file_path)

		exit()
	except KeyboardInterrupt:
		pass
