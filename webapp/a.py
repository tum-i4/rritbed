#!/usr/bin/env python

import argparse
from ids.dir_utils import Dir


def a(file_path):
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


if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		PARSER.add_argument("file_path", metavar="PATH/FILE", help="Log file")

		ARGS = PARSER.parse_args()

		a(ARGS.file_path)

		exit()
	except KeyboardInterrupt:
		pass
