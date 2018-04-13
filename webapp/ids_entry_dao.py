#!/usr/bin/env python
""" Convenient access to IDS entries, stored in various forms. """

import os


def get_entries(file_path, limit=None):
	""" Get a generator of IdsEntry objects from the given file.
	First retrieval might be slower than others. """

	if not os.path.lexists(file_path):
		raise IOError("File not found: %s" % file_path)

	raise NotImplementedError()


def save_entries(file_path, entries):
	""" Store the entries as a IDSE file. """

	if os.path.lexists(file_path):
		raise IOError("Output file exists already: %s" % file_path)

	raise NotImplementedError()
