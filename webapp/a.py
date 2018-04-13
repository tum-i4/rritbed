#!/usr/bin/env python
""" a """

# pylint: disable-msg=C0103; (Invalid name)

import argparse

import sklearn
import sklearn.ensemble as sk_ens
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


ITEM_LIMIT = 5000000


### Workers ###


def a(file_path):
	""" Read entries, convert them, split them per app_id and call b() for each app. """

	printer = TimePrinter(name="a")
	printer.prt("Reading file and converting...")

	# ids_entries: { app_id, vector, my_class }
	import time
	start_time = time.time()
	ids_entries_per_app = read_sample_convert(file_path)
	resaco_time = time.time()
	LOL = read_convert(file_path)
	reco_time = time.time()

	print("TIME FOR RESACO: %s" % util.fmtr.format_time_passed(resaco_time - start_time))
	print("TIME FOR RECO__: %s" % util.fmtr.format_time_passed(reco_time - resaco_time))
	exit()

	handle_all(ids_entries_per_app)

	for app_id, ids_entries in ids_entries_per_app.items():
		handle_app(app_id, ids_entries)


def handle_all(ids_entries_per_app):
	""" Full flow for a one-fits-all classifier. """

	print("ALLLLLLLL\n\n")

	ids_entries = []

	# Dict to list
	for _, entries in ids_entries_per_app:
		entries = ids_tools.straighten_dataset_for_app(ids_entries)
		ids_entries.extend(entries)

	training_entries, scoring_entries = ids_tools.ids_entries_to_train_test(ids_entries)
	X_train, _ = TEMP_IDS_ENTRIES_TO_X_Y(training_entries)

	scoring_dict = ids_tools.empty_app_id_to_list_dict()
	for ids_entry in scoring_entries:
		scoring_dict[ids_entry.app_id] = ids_entry

	# Classify with all entries
	# training_entries
	classifier = sklearn.svm.OneClassSVM()
	classifier.fit(X_train)

	for app_id, app_entries in scoring_dict.items():
		X_test, y_true = TEMP_IDS_ENTRIES_TO_X_Y(app_entries)
		y_pred = classifier.predict(X_test)
		visualise(app_id, y_true, y_pred)

	print("\n\n\nSPLIT PER APPPPPPPP\n\n")


def TEMP_IDS_ENTRIES_TO_X_Y(ids_entries):
	""" Convert the given IdsEntry objects to (X, y). """

	# pylint: disable-msg=C0103; (Invalid variable name)
	X = []
	y = []

	for ids_entry in ids_entries:
		X.append(ids_entry.vector)
		y.append(ids_entry.vclass)

	return (X, y)


def handle_app(app_id, ids_entries):
	""" Full flow for one classifier. """

	printer = TimePrinter(name=app_id)

	if not ids_entries:
		printer.prt("No input data for {}".format(app_id))
		return

	if not isinstance(ids_entries[0], IdsEntry):
		raise TypeError("Given list does not contain IdsEntry objects.")

	ids_entries = ids_tools.straighten_dataset_for_app(ids_entries)

	print("\n\t::: %s :::\n" % app_id)

	# TODO
	_, _ = preprocess_fit_score(app_id, ids_entries,
		lambda x: x,
		sklearn.svm.OneClassSVM(),
		printer)

	name = "IF"
	n_est = 100
	max_sampl = 256
	print("\n\t> %s - n_est: %s, max_sampl: %s" % (name, n_est, max_sampl))
	_, _ = preprocess_fit_score(app_id, ids_entries,
		lambda x: x,
		sk_ens.IsolationForest(n_estimators=n_est, max_samples=max_sampl, n_jobs=-1, random_state=0),
		printer)

	# _, _ = preprocess_fit_score(app_id, ids_entries,
	# 	lambda x: sk_pre.scale(x),
	# 	sklearn.svm.OneClassSVM(),
	# 	printer)


def preprocess_fit_score(app_id, ids_entries, preprocessor, classifier, printer):
	""" Use the given preprocessor on the data, classify it with the given classifier and score. """

	converter = IdsConverter()
	X, y = converter.ids_entries_to_X_y(app_id, ids_entries)

	printer.prt("Preprocessing... ", newline=False)
	X = preprocessor(X)

	printer.prt("Splitting... ", newline=False)
	X_train, _, X_test, y_true = ids_tools.X_y_to_train_test(X, y)

	printer.prt("Fitting... ", newline=False)
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


def e(app_id, ids_entries):

	if app_id not in ids_data.get_generators():
		return

	import numpy as np
	from sklearn.covariance import EllipticEnvelope
	from sklearn.svm import OneClassSVM
	import matplotlib.pyplot as plt
	import matplotlib.font_manager

	X_list, y = IdsConverter().ids_entries_to_X_y(app_id, ids_entries)
	X = np.array(X_list)

	X_scaled = sk_pre.scale(X)

	print("WTF\n")


### Helpers ###


def read_convert(file_path):
	""" Read log entries from the given file and convert the result. """

	converter = IdsConverter()
	log_entries = []

	for line in Dir.yield_lines(file_path, ITEM_LIMIT):
		log_entry = LogEntry.from_log_string(line)
		log_entries.append(log_entry)

	ids_entries_dict = converter.log_entries_to_ids_entries_dict(log_entries)
	return ids_entries_dict


def read_sample_convert(file_path):
	""" Sample log entries from the given file and convert the result. """

	converter = IdsConverter()
	log_entry_generator = (LogEntry.from_log_string(line) for line in Dir.yield_lines(file_path))
	log_entries = ids_tools.reservoir_sample(log_entry_generator, ITEM_LIMIT)

	assert(len(log_entries)) == ITEM_LIMIT

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
