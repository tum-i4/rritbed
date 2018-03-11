#!/usr/bin/env python
""" Tools for command-line interaction with the server """

import argparse
import os

from log_entry import LogEntry
from ids.intrusion_classifier import IntrusionClassifier
from ids.dir_utils import Dir


def train_call(args):
	""" Unpack the args and call _train. """
	_train(args.train_file_path, args.extend_models)


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
	lines = Dir.read_lines(file_path)
	print("File read. Converting to LogEntry objects...")
	# Remove newline at the end of the line and create LogEntry objects
	log_entries = [LogEntry.from_log_string(line[:-1]) for line in lines]
	print("Done.")

	clas = IntrusionClassifier.get_singleton()
	try:
		clas.train(log_entries, extend_models=extend_models)
	except ValueError as val_err:
		print(val_err.message)
		exit()



if __name__ == "__main__":

	PARSER = argparse.ArgumentParser()
	SUBPARSERS = PARSER.add_subparsers()

	TRAIN_PARSER = SUBPARSERS.add_parser("train", help="Train the classifier")
	TRAIN_PARSER.add_argument("train_file_path", metavar="FILE_PATH",
		help="File to train the classifier with.")
	TRAIN_PARSER.add_argument("--extend-models", "-e", action="store_true", dest="extend_models",
		help="Allow existing models to be extended.")
	TRAIN_PARSER.set_defaults(function=train_call)

	ARGS = PARSER.parse_args()

	ARGS.function(ARGS)
