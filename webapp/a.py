#!/usr/bin/env python
""" a """

# pylint: disable-msg=C0103; (Invalid name)

import argparse

import sklearn
import sklearn.metrics as sk_metr
import sklearn.preprocessing as sk_pre

from log_entry import LogEntry
import ids.ids_data as ids_data
import ids.ids_tools as ids_tools
from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter
from ids.ids_entry import IdsEntry
import util.fmtr
import util.outp
from util.prtr import TimePrinter


### Workers ###


def a(file_path):
	""" Read entries, convert them, split them per app_id and call b() for each app. """

	printer = TimePrinter(name="a")
	printer.prt("Reading file and converting...")

	# ids_entries: { app_id, vector, my_class }
	ids_entries_per_app = read_convert(file_path)

	for app_id, ids_entries in ids_entries_per_app.items():
		handle_app(app_id, ids_entries)


def handle_app(app_id, ids_entries):
	""" Full flow for one classifier. """

	printer = TimePrinter(name=app_id)

	if not ids_entries:
		printer.prt("No input data for {}".format(app_id))
		return

	if not isinstance(ids_entries[0], IdsEntry):
		raise TypeError("Given list does not contain IdsEntry objects.")

	raise NotImplementedError()


def preprocess_fit_score(app_id, ids_entries, preprocessor, classifier, printer):

	converter = IdsConverter()
	X, y = converter.ids_entries_to_X_y(app_id, ids_entries)

	printer.prt("Preprocessing... ", newline=False)
	X = preprocessor(X)

	printer.prt("Splitting... ", newline=False)
	X_train, _, X_test, y_true = ids_tools.X_y_to_train_test(X, y)

	printer.prt("Fitting... ", newline=False)
	classifier = sklearn.svm.OneClassSVM(kernel="linear", random_state=0)
	classifier.fit(X_train)

	printer.prt("Predicting... ")
	y_pred = classifier.predict(X_test)

	visualise(app_id, y_true, y_pred)

	return (y_true, y_pred)


def visualise(app_id, y_true, y_pred):
	""" Score, print. """

	tn, fp, fn, tp = sk_metr.confusion_matrix(y_true, y_pred).ravel()

	table = []
	table.append(["", "Actual (+)", "Actual (-)"])
	table.append(["Pred (+)", tp, fp])
	table.append(["Pred (-)", fn, tn])
	util.outp.print_table(table)


### Helpers ###


def read_convert(file_path):
	""" Read log entries from the given file and convert the result. """

	converter = IdsConverter()
	log_entries = []

	for line in Dir.yield_lines(file_path, 5000000):
		log_entry = LogEntry.from_log_string(line)
		log_entries.append(log_entry)

	ids_entries_dict = converter.log_entries_to_ids_entries_dict(log_entries)
	return ids_entries_dict


if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		PARSER.add_argument("file_path", metavar="PATH/FILE", help="Log file")

		ARGS = PARSER.parse_args()

		a(ARGS.file_path)

		exit()
	except KeyboardInterrupt:
		pass
