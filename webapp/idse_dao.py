#!/usr/bin/env python
""" Convenient access to IDS entries, stored in various forms. """

import os
from collections import namedtuple
from enum import Enum

from log_entry import LogEntry
from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter


#### IDSE FILE SPECIFICATION
#### (aka glorified CSV)
####
#### The first line in each file is the header (see CSV):
HEADER = "IDSE_V_1(APP_ID,FEATURE_COUNT,VCLASS,FEATURES)"
####
#### Each line contains, comma-separated:
#### 1   app_id : str
#### 1   feature_count : int
#### 1   vclass : [-1, 1]
#### 1-X feature : float

# Definitions
MIN_ELEMENTS_PER_LINE = 4
# Expected types of fields; the last element is the type for all remaining feature fields.
ELEMENT_TYPES = [str, int, int, float]
# Verifiers can (but don't have to be) defined; each index will be checked with the given lambda.
Verifier = namedtuple("Verifier", "element_index, verify_lambda")
VERIFIER = [
	Verifier(element_index=1, verify_lambda=lambda x: x > 0),
	Verifier(element_index=2, verify_lambda=lambda x: x in [-1, 1])
]


def get_entries(file_path, limit=None):
	"""
	Yield IdsEntry objects from the given file. First access on log files is costly!
	*limit: Optional maximum number of entries to retrieve.
	"""

	if not os.path.lexists(file_path):
		raise IOError("File not found: %s" % file_path)

	yielder = Dir.yield_lines(file_path, limit)

	first_line = yielder.next()
	file_type = _detect_type(first_line)

	if file_type == FileType.IDSE_FILE:
		_yield_idse_lines(yielder)
	elif file_type == FileType.LOG_FILE:
		_read_log_lines_then_yield(yielder, first_line)
	else:
		raise NotImplementedError("File type not implemented: %s" % file_type)


def save_entries(file_path, entries):
	""" Store the entries as a IDSE file. """

	if os.path.lexists(file_path):
		raise IOError("Output file exists already: %s" % file_path)

	raise NotImplementedError()


def _detect_type(first_line):
	""" Detect the file type from the first line. """

	if first_line == HEADER:
		return FileType.IDSE_FILE

	try:
		_ = LogEntry.from_log_string(first_line)
		return FileType.LOG_FILE
	except ValueError:
		raise ValueError("Invalid file given! Can't parse:\n%s" % first_line)


def _yield_idse_lines(yielder):
	raise NotImplementedError()


def _read_log_lines_then_yield(yielder, first_line):
	""" Read all provided log lines from the given yielder. """

	first_entry = LogEntry.from_log_string(first_line)
	log_entries = [first_entry]
	for line in yielder:
		log_entry = LogEntry.from_log_string(line)
		log_entries.append(log_entry)

	ids_entry_dict = IdsConverter().log_entries_to_ids_entries_dict(log_entries)

	for _, app_entries in ids_entry_dict.items():
		for ids_entry in app_entries:
			yield ids_entry



# pylint: disable-msg=R0903, C0111; (Too few public methods - enum, missing class docstring - obvious)
class FileType(Enum):
	LOG_FILE = 0
	IDSE_FILE = 1
